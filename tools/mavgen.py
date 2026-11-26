#!/usr/bin/env python3

'''
parse a MAVLink protocol XML file and generate a python implementation

Copyright Andrew Tridgell 2011
Released under GNU GPL version 3 or later

'''

# allow running mavgen from within the tree without installing
if __name__ == "__main__" and __package__ is None:
    from os import sys, path
    sys.path.insert(0, path.dirname(path.dirname(path.dirname(path.abspath(__file__)))))

from pymavlink.generator import mavgen
from pymavlink.generator import mavparse

from argparse import ArgumentParser

parser = ArgumentParser(description="This tool generate implementations from MAVLink message definitions")
parser.add_argument("-o", "--output", default="mavlink", help="output directory.")
parser.add_argument("--lang", dest="language", choices=mavgen.supportedLanguages, default=mavgen.DEFAULT_LANGUAGE, help="language of generated code [default: %(default)s]")
parser.add_argument("--wire-protocol", choices=[mavparse.PROTOCOL_1_0, mavparse.PROTOCOL_2_0], default=mavgen.DEFAULT_WIRE_PROTOCOL, help="MAVLink protocol version. [default: %(default)s]")
parser.add_argument("--no-validate", action="store_false", dest="validate", default=mavgen.DEFAULT_VALIDATE, help="Do not perform XML validation. Can speed up code generation if XML files are known to be correct.")
parser.add_argument("--strict-units", action="store_true", dest="strict_units", default=mavgen.DEFAULT_STRICT_UNITS, help="Perform validation of units attributes.")
parser.add_argument("--cs-layout", choices=["legacy", "split"], default="legacy", help="C# only: 'legacy' is the single mavlink.cs (unchanged); 'split' is one file and namespace per XML file, for MavLink.Net. [default: %(default)s]")
parser.add_argument("--cs-emit", choices=["root", "all"], default="all", help="C# split layout only: generate the root XML file only, or every file in its include closure. [default: %(default)s]")
parser.add_argument("--cs-namespace-root", default="MavLink.Net", help="C# split layout only: namespace root for dialects without a <namespace> element: <root>.Dialects.<Name>. [default: %(default)s]")
parser.add_argument("--cs-namespace", default=None, help="C# split layout only: namespace for the root XML file, overriding its <namespace> element. Included files keep their own <namespace> or the default rule.")
parser.add_argument("--cs-check-against", action="append", metavar="XML", default=None, help="C# split layout only (repeatable): another dialect XML file (with its includes) to check for message id and MAV_CMD collisions. Collisions are printed as warnings; generation still succeeds.")
parser.add_argument("definitions", metavar="XML", nargs="+", help="MAVLink definitions")
args = parser.parse_args()

ok = mavgen.mavgen(args, args.definitions)
# report failure in the exit code when no exception is thrown
if not ok:
    exit(1)
