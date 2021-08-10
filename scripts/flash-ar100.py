#!/usr/bin/env python3
# This file may be distributed under the terms of the GNU GPLv3 license.
import mmap
import argparse
import sys

EXCEPTIONS_BASE = 0x00040000
EXCEPTIONS_LIMIT = 0x00042000
EXCEPTIONS_SIZE = EXCEPTIONS_LIMIT - EXCEPTIONS_BASE
EXCEPTIONS_JUMP = 0x00009000  # All exceptions reset program
NR_OF_EXCEPTIONS = 14

SRAM_A2_BASE = 0x00049000
SRAM_A2_LIMIT = 0x00054000
SRAM_A2_SIZE = SRAM_A2_LIMIT - SRAM_A2_BASE

R_CPU_CFG_PAGE_BASE = 0x01F01000
R_CPU_CFG_PAGE_LIMIT = 0x01F02000
R_CPU_CFG_SIZE = R_CPU_CFG_PAGE_LIMIT - R_CPU_CFG_PAGE_BASE
R_CPU_CFG_OFFSET = 0xC00

parser = argparse.ArgumentParser(
    description='Write AR100 binary file to SRAM of A64')
parser.add_argument(
    'filename', nargs='?', help='binary file to write to memory')
parser.add_argument(
    '--reset', action='store_true', help='reset the AR100 cpu and exit')
args = parser.parse_args()


def write_exception_vectors():
    print("Writing exception vectors")
    with open("/dev/mem", "w+b") as f:
        exc = mmap.mmap(
            f.fileno(), length=EXCEPTIONS_SIZE, offset=EXCEPTIONS_BASE)
        for i in range(NR_OF_EXCEPTIONS):
            add = i * 0x100
            exc[add:add + 4] = ((EXCEPTIONS_JUMP - add) >> 2).to_bytes(
                4, byteorder='little')
        exc.close()


def assert_deassert_reset(ass):
    with open("/dev/mem", "w+b") as f:
        r_cpucfg = mmap.mmap(
            f.fileno(), length=R_CPU_CFG_SIZE, offset=R_CPU_CFG_PAGE_BASE)
        if ass:
            r_cpucfg[R_CPU_CFG_OFFSET] &= ~0x01
            if r_cpucfg[R_CPU_CFG_OFFSET] & 0x01:
                print("failed to assert reset")
        else:
            r_cpucfg[R_CPU_CFG_OFFSET] |= 0x01
            if not (r_cpucfg[R_CPU_CFG_OFFSET] & 0x01):
                print("failed to deassert reset")
        r_cpucfg.close()


def write_file(filename):
    with open(filename, "r+b") as fw:
        data = fw.read()
        if len(data) > SRAM_A2_SIZE:
            print("File does not fit in memory")
            assert_deassert_reset(0)
            sys.exit(1)
        print("Writing file to SRAM A2")
        with open("/dev/mem", "w+b") as f:
            sram_a2 = mmap.mmap(
                f.fileno(), length=SRAM_A2_SIZE, offset=SRAM_A2_BASE)
            sram_a2[0:len(data)] = data
            sram_a2.close()


if args.reset:
    print("Resetting AR100")
    assert_deassert_reset(1)
    assert_deassert_reset(0)
    sys.exit(0)

if args.filename:
    assert_deassert_reset(1)
    write_exception_vectors()
    write_file(args.filename)
    assert_deassert_reset(0)
