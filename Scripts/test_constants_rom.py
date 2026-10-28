#!/usr/bin/env python3
"""Check packed CROM boundaries and every forward/inverse seed against modular powers."""
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

from fhe_profiles import PRIMES, read_coe, rns_words, stride

SCRIPTS = Path(__file__).resolve().parent


def words(path):
    return [int(x, 16) for x in read_coe(path)]


class PackedConstantsTest(unittest.TestCase):
    def test_rtl_catalog(self):
        """fhe_params_pkg.sv's {k, qm} table must spell the same primes."""
        pkg = SCRIPTS.parents[2] / 'rtl/fhe_params_pkg.sv'
        if not pkg.is_file():
            self.skipTest('not inside the caliptra-rtl tree')
        import re
        rows = re.findall(r"4'd(\d+):\s*fhe_kqm = \{4'd(\d+), 17'd(\d+)\}|default: fhe_kqm = \{4'd(\d+), 17'd(\d+)\}",
                          pkg.read_text())
        q = {}
        for i, k, qm, dk, dqm in rows:
            idx, k, qm = (int(i), int(k), int(qm)) if i else (len(PRIMES) - 1, int(dk), int(dqm))
            q[idx] = (1 << (46 + k)) - (qm << 24) + 1
        self.assertEqual(tuple(q[i] for i in range(len(PRIMES))), PRIMES)

    def test_profiles_and_seeds(self):
        with tempfile.TemporaryDirectory() as directory:
            for h, profiles in ((h,p) for h in (7,8,13,14) for p in (1,2,4,8,15)):
                with self.subTest(logn=h, profiles=profiles):
                    output = Path(directory) / f'n{1 << h}.coe'
                    subprocess.run([sys.executable, str(SCRIPTS / 'GenerateConstantsROM.py'), str(h), str(output), "--profiles", str(profiles)], check=True)
                    image = words(output)
                    width = stride(1 << h)
                    self.assertEqual(len(image), 128 + profiles * width)
                    # The RNS region starts right after the FFT prefix.
                    for i, q in enumerate(PRIMES[:profiles]):
                        self.assertEqual(tuple(image[width+2*i:width+2*i+2]), rns_words(q))
                    self.assertFalse(any(image[width+2*profiles:128]))
                    # Each stage seeds the first sixteen butterfly positions;
                    # repeated factors occupy only one ROM word.
                    exponents = [((2 * j + 1) << stage)
                                 for stage in range(h)
                                 for j in range(max(1, 16 >> stage))]
                    for profile, q in enumerate(PRIMES[:profiles]):
                        base = 128 + profile * width
                        block = image[base:base + width]
                        r = pow(2, 72, q)
                        mask = (1 << 54) - 1
                        root = (block[0] & mask) * pow(r, -1, q) % q
                        self.assertEqual(pow(root, 1 << h, q), q - 1,
                                         f'profile {profile} must start with a primitive-root seed, not padding')
                        for entry, exponent in zip(block, exponents):
                            self.assertEqual(entry & mask, pow(root, exponent, q) * r % q)
                            self.assertEqual(entry >> 54, pow(root, -exponent, q) * r % q)


if __name__ == '__main__':
    unittest.main()
