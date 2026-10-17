"""Append-only simulation evidence and atomic progress files."""
import json
from pathlib import Path


def write_json(path: Path, data):
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(json.dumps(data, indent=2, ensure_ascii=False), encoding='utf-8')
    temporary.replace(path)


def append_json(path: Path, data):
    with path.open('a', encoding='utf-8') as stream:
        stream.write(json.dumps(data, ensure_ascii=False) + '\n')


class SolverRecords:
    """Copy each native solve once; never repeatedly rewrite an expanding log."""
    def __init__(self, native, destination):
        self.native = Path(native)
        self.destination = Path(destination)
        self.offset = 0
        self.summary = {
            'linear_solves': 0, 'fallbacks': 0,
            'max_schur_relative_residual': 0., 'max_full_relative_residual': 0.,
            'max_pcg_iterations': 0, 'max_mesh_contacts': 0,
            'max_ground_contacts': 0, 'max_aca_error': 0.,
            'min_rank': None, 'max_rank': 0,
            'min_neumann_doublings': None, 'max_neumann_doublings': 0,
        }

    def collect(self):
        rows = []
        if not self.native.exists():
            return rows
        with self.native.open('rb') as source, self.destination.open('ab') as target:
            source.seek(self.offset)
            while line := source.readline():
                if not line.endswith(b'\n'):
                    break
                row = json.loads(line)
                target.write(line)
                self.offset = source.tell()
                rows.append(row)
                s = self.summary
                s['linear_solves'] += 1
                s['fallbacks'] += int(row['fallback'])
                if row['solver'] != 'linear_hsc' or row['fallback'] or not row['accepted']:
                    raise RuntimeError('Native HSC returned an unaccepted solve or fallback')
                for output, source_key in (
                    ('max_schur_relative_residual', 'schur_relative_residual'),
                    ('max_full_relative_residual', 'full_relative_residual'),
                    ('max_pcg_iterations', 'pcg_iterations'),
                    ('max_mesh_contacts', 'active_mesh_contacts'),
                    ('max_ground_contacts', 'active_ground_contacts'),
                    ('max_aca_error', 'aca_relative_error'),
                    ('max_rank', 'rank'), ('max_neumann_doublings', 'neumann_doublings'),
                ):
                    s[output] = max(s[output], row[source_key])
                for output, source_key in (('min_rank', 'rank'), ('min_neumann_doublings', 'neumann_doublings')):
                    s[output] = row[source_key] if s[output] is None else min(s[output], row[source_key])
        return rows
