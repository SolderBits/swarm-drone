"""Tiny S-expression reader/writer for KiCad files."""
import re

_tok = re.compile(r'\s*(?:(\()|(\))|("(?:[^"\\]|\\.)*")|([^\s()"]+))')


class Sym(str):
    """Bare (unquoted) atom."""


def parse(text):
    pos = 0
    stack = [[]]
    while True:
        m = _tok.match(text, pos)
        if not m or m.end() == pos:
            break
        pos = m.end()
        o, c, s, a = m.groups()
        if o:
            stack.append([])
        elif c:
            node = stack.pop()
            stack[-1].append(node)
        elif s is not None:
            stack[-1].append(re.sub(r'\\(.)', lambda m: {'n': '\n', 't': '\t'}.get(m.group(1), m.group(1)), s[1:-1]))
        elif a is not None:
            stack[-1].append(Sym(a))
    return stack[0][0]


def _atom(x):
    if isinstance(x, Sym):
        return str(x)
    if isinstance(x, bool):
        return 'yes' if x else 'no'
    if isinstance(x, (int, float)):
        if isinstance(x, float):
            s = f'{x:.4f}'.rstrip('0').rstrip('.')
            return s if s not in ('-0', '') else '0'
        return str(x)
    s = str(x).replace('\\', '\\\\').replace('"', '\\"').replace('\n', '\\n')
    return f'"{s}"'


def dumps(node, indent=0):
    if not isinstance(node, list):
        return _atom(node)
    simple = all(not isinstance(c, list) for c in node)
    if simple and len(node) < 12:
        return '(' + ' '.join(_atom(c) for c in node) + ')'
    parts = []
    head = []
    for c in node:
        if isinstance(c, list):
            break
        head.append(_atom(c))
    rest = node[len(head):]
    s = '(' + ' '.join(head)
    for c in rest:
        s += '\n' + '  ' * (indent + 1) + dumps(c, indent + 1)
    s += ')'
    return s


def find(node, key):
    for c in node:
        if isinstance(c, list) and c and c[0] == key:
            return c
    return None


def findall(node, key):
    return [c for c in node if isinstance(c, list) and c and c[0] == key]
