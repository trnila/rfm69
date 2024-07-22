#!/usr/bin/env python3
import multiprocessing
import clang.cindex
from dataclasses import dataclass
from clang.cindex import CursorKind
import struct
import json
import os
import signal
import argparse
import subprocess
import tempfile
from pathlib import Path
import usb.core

CONFIG_ADDR = 0x0800F800

@dataclass
class Field:
    name: str
    type: str
    len: int = 1

@dataclass
class Target:
    serial: str
    executable: str
    name: str
    connected: bool = False

def openocd_args(serial):
    return  [
        'openocd',
        '-f', 'interface/stlink.cfg',
        '-f', 'target/stm32g0x.cfg',
        '-c', f'hla_serial {serial}',
        '-c', 'reset_config srst_only srst_nogate connect_assert_srst',
        '-c', 'init; reset halt',
    ]


def openocd(serial, command):
    subprocess.check_call([
        *openocd_args(serial),
        '-c', command,
        '-c', 'reset run;shutdown',
    ])

def extract_definition(cursor):
    if not cursor.location.file:
        return ""

    filename = cursor.location.file.name
    with open(filename, 'r') as fh:
        contents = fh.read()
    return contents[cursor.extent.start.offset: cursor.extent.end.offset]

class StructParser:
    def __init__(self):
        self.level = 0

    def parser(fn):
        def wrap(self, node):
            self.level += 1
            print(' ' * self.level, node.kind, extract_definition(node))
            res = fn(self, node)
            self.level -= 1
            return res
        return wrap

    def parse_struct(self, node):
        if node.kind == CursorKind.TYPEDEF_DECL:
            node = next(node.get_children())

        fields = []
        for field_node in node.get_children():
            desc = list(field_node.get_children())
            count = 1 if len(desc) == 1 else desc[1].spelling
            if count != 1:
                count = int(next(desc[1].get_tokens()).spelling)
            fields.append(Field(field_node.spelling, desc[0].spelling, count))

        return {node.spelling: fields}

    def parse(self, node):
        if node.kind == CursorKind.TYPEDEF_DECL and node.spelling in ('Radio', 'Config'):
            return self.parse_struct(node)

        structs = {}
        for child in node.get_children():
            structs.update(self.parse(child))
        return structs

map = {
    "uint8_t": "B",
    "int8_t" : "b",
    "bool": "?",
    "uint16_t": "H",
    "int16_t": "h",
    "uint32_t": "I",
    "int32_t": "i",
    "uint64_t": "Q",
    "int64_t": "q",
}

def generate_config(value, datatype, structs):
    if datatype in map:
        return struct.pack(map[datatype], value)

    result = bytearray()
    for field in structs[datatype]:
        if field.len == 1:
            result += generate_config(value[field.name], field.type, structs)
        else:
            for val in value[field.name]:
                result += generate_config(val, field.type, structs)
    return result

def parse_config(bin_value, datatype, structs):
    if datatype in map:
        size = struct.calcsize(map[datatype])
        return struct.unpack(map[datatype], bin_value[:size])[0], bin_value[size:]

    result = {}
    for field in structs[datatype]:
        if field.len == 1:
            value, bin_value = parse_config(bin_value, field.type, structs)
            result[field.name] = value
        else:
            result[field.name] = []
            for i in range(field.len):
                value, bin_value = parse_config(bin_value, field.type, structs)
                result[field.name].append(value)
    return result, bin_value

def all_targets() -> list[Target]:
    with open(".vscode/launch.json") as f:
            launch = json.load(f)
    targets = {}
    for conf in launch['configurations']:
        for opt in conf['openOCDLaunchCommands']:
            match opt.split(' '):
                case ['hla_serial', serial]:
                    targets[conf['name']] = Target(
                        executable=Path(conf['executable']).stem,
                        name=conf['name'],
                        serial=serial,
                    )

    targets_serial = {t.serial: t for t in targets.values()}
    unknown = 0
    for dev in usb.core.find(idVendor=0x483, idProduct=0x374b, find_all=True):
        target = targets_serial.get(dev.serial_number, None)
        if target:
            target.connected = True
        else:
            name = f'unknown_{unknown}'
            unknown += 1
            targets[name] = Target(executable=None, name=name, serial=dev.serial_number, connected=True)

    return targets

class Module:
    def args(self, parser: argparse.ArgumentParser):
        pass

    def run(self, args):
        raise NotImplementedError

class TargetModule(Module):
    def args(self, parser: argparse.ArgumentParser):
        parser.add_argument('--all', '-a', action='store_true')

    def run(self, args):
        for dev in all_targets().values():
            print(dev)

class ParamsModule(Module):
    def args(self, parser: argparse.ArgumentParser):
        subparsers = parser.add_subparsers(dest='action', required=True)

        p = subparsers.add_parser('read', help='Read calibrations from the target')
        p.add_argument('target')

        p = subparsers.add_parser('edit', help='Read calibrations, edit and write them to the target')
        p.add_argument('target')


    def run(self, args):
        target = all_targets()[args.target]

        index = clang.cindex.Index.create()
        tu = index.parse(Path("src") / f"{target.executable}.c")
        structs = StructParser().parse(tu.cursor)

        def read_config():
            with tempfile.NamedTemporaryFile('rb') as f:
                openocd(target.serial, f'flash read_bank 0 {f.name} {CONFIG_ADDR - 0x08000000}')
                raw_data = f.read()
                return parse_config(raw_data, 'Config', structs)[0]

        def write_config(config):
            with tempfile.NamedTemporaryFile('wb') as f:
                f.write(generate_config(config, 'Config', structs))
                f.flush()
                openocd(target.serial, f'flash write_image erase {f.name} {CONFIG_ADDR} bin')

        if args.action == 'read':
            print(json.dumps(read_config(), indent=2, sort_keys=True))
        elif args.action == 'edit':
            config = read_config()
            with tempfile.NamedTemporaryFile('r+', suffix='.json') as f:
                json.dump(config, f, indent=2, sort_keys=True)
                f.flush()
                subprocess.run(["vim", f.name])
                f.seek(0)
                config = json.load(f)

            write_config(config)
        else:
            raise NotImplementedError

def build():
    try:
        subprocess.check_call(['make', '-j', str(multiprocessing.cpu_count())])
    except subprocess.CalledProcessError:
        print("BUILD failed")
        exit(1)

class FlashModule(Module):
    def args(self, parser: argparse.ArgumentParser):
        parser.add_argument('target', nargs='*')

    def run(self, args):
        build()

        targets = all_targets()
        unknown_targets = args.target - targets.keys()
        if unknown_targets:
            print("Unknown targets:", *unknown_targets)
            exit(1)

        results = {}
        for target in targets.values():
            if not args.target or target.name in args.target:
                result = 'Disconnected'
                if target.connected:
                    try:
                        openocd(target.serial, f'flash write_image erase build/{target.executable}.hex')
                        result = 'OK'
                    except subprocess.CalledProcessError as e:
                        result = 'FAILED'
                        print(e)
                results[target.name] = result

        print()
        for name, result in results.items():
            print(f"{result: >13} {name}")

class GDBModule(Module):
    def args(self, parser: argparse.ArgumentParser):
        parser.add_argument('target')

    def run(self, args):
        build()

        targets = all_targets()
        target = targets[args.target]

        p_ocd = subprocess.Popen(openocd_args(target.serial), preexec_fn=os.setsid)
        try:
            p_gdb = subprocess.Popen([
                'arm-none-eabi-gdb',
                '-q',
                f'build/{target.executable}.elf',
                '-ex', 'target extended-remote :3333'
            ], preexec_fn=os.setsid)
            signal.signal(signal.SIGINT, lambda *_: p_gdb.send_signal(signal.SIGINT))
            p_gdb.wait()
        finally:
            p_gdb.terminate()
            p_ocd.terminate()

parser = argparse.ArgumentParser()
subparsers = parser.add_subparsers(dest='module', required=True)

modules = {var.__name__.replace('Module', '').lower(): var() for var in globals().values() if type(var) == type and issubclass(var, Module) and var != Module}
for name, t in modules.items():
    t.args(subparsers.add_parser(name))

args = parser.parse_args()
modules[args.module].run(args)
