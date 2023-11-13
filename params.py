#!/usr/bin/env python3
import clang.cindex
from dataclasses import dataclass
from clang.cindex import CursorKind
import struct
import json
import argparse
import subprocess
import tempfile
from pathlib import Path

CONFIG_ADDR = 0x0800F800

@dataclass
class Field:
    name: str
    type: str
    len: int = 1

def openocd(command):
    subprocess.check_call([
        'openocd',
        '-f', 'interface/stlink.cfg',
        '-f', 'target/stm32g0x.cfg',
        '-c', 'init; reset halt',
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

def generate_config(value, datatype):
    if datatype in map:
        return struct.pack(map[datatype], value)

    result = bytearray()
    for field in finals[datatype]:
        if field.len == 1:
            result += generate_config(value[field.name], field.type)
        else:
            for val in value[field.name]:
                result += generate_config(val, field.type)
    return result

def parse_config(bin_value, datatype):
    if datatype in map:
        size = struct.calcsize(map[datatype])
        return struct.unpack(map[datatype], bin_value[:size])[0], bin_value[size:]

    result = {}
    for field in finals[datatype]:
        if field.len == 1:
            value, bin_value = parse_config(bin_value, field.type)
            result[field.name] = value
        else:
            result[field.name] = []
            for i in range(field.len):
                value, bin_value = parse_config(bin_value, field.type)
                result[field.name].append(value)
    return result, bin_value

parser = argparse.ArgumentParser()
parser.add_argument('app')
subparsers = parser.add_subparsers(dest='action', required=True)
p = subparsers.add_parser('write')
p.add_argument('node_id', type=int)
p.add_argument('params', nargs='?')
subparsers.add_parser('read')
subparsers.add_parser('edit')
args = parser.parse_args()

index = clang.cindex.Index.create()
tu = index.parse(Path("src") / f"{args.app}.c")
finals = StructParser().parse(tu.cursor)

def merge(a, b):
    for key in b:
        if key in a:
            merge(a[key], b[key])
        else:
            a[key] = b[key]
    return a


def read_config():
    with tempfile.NamedTemporaryFile('rb') as f:
        openocd(f'flash read_bank 0 {f.name} {CONFIG_ADDR - 0x08000000}')
        raw_data = f.read()
        return parse_config(raw_data, 'Config')[0]

def write_config(config):
    with tempfile.NamedTemporaryFile('wb') as f:
        f.write(generate_config(config, 'Config'))
        f.flush()
        openocd(f'flash write_image erase {f.name} {CONFIG_ADDR} bin')

if args.action == 'write':
    with open("secret.json") as f:
        config = json.load(f)
        config['radio']['node_id'] = args.node_id

    if args.params:
        with open(args.params) as f:
            config = merge(config, json.load(f))

    print(json.dumps(config, indent=2, sort_keys=True))
    write_config(config)
elif args.action == 'read':
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
