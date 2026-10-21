"""Ordered prime catalog shared by RNS and NTT; installed profiles are a prefix.

Catalog 1 uses q_k = 2^(46+k) - qm_k*2^24 + 1, k=0..7.
A build installs the first L_max = 1..8 profiles.
"""
from pathlib import Path

PRIMES = (0x3ffff7000001, 0x7fffff000001, 0xffffd8000001, 0x1fffffb000001,
          0x3ffffe4000001, 0x7fffff9000001, 0xffffff3000001, 0x1ffffff6000001)
CATALOG_ID = 1
LAYOUT_VERSION = 2
MAGIC = 0x46484532  # "FHE2", first word of the firmware CROM image manifest


def installed(count):
    if not 1 <= count <= len(PRIMES):
        raise ValueError('installed profile count must be 1..8')
    return PRIMES[:count]


def stride(n):
    """Words per NTT profile; also the first RNS word after the FFT prefix."""
    return n.bit_length() - 1 + 26


def geometry(n, count):
    installed(count)
    if n < 128 or n > 16384 or n & (n - 1):
        raise ValueError('supported N is a power of two in 128..16384')
    words = 128 + count * stride(n)
    return words, 1 << (words - 1).bit_length()


def rns_words(q):
    """The two packed CROM words holding 2^(144+40k) mod q, k=0..3."""
    c = [pow(2, 144 + shift, q) for shift in (0, 40, 80, 120)]
    return c[0] | (c[1] << 54), c[2] | (c[3] << 54)


def read_coe(path):
    """Hex words of a Xilinx .coe file (the vector after the last '=')."""
    return Path(path).read_text().split('=')[-1].replace(';', ' ').replace(',', ' ').split()
