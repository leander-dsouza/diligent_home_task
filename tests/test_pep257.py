#! /usr/bin/env python3
"""Script to analyse pep257 linting errors in the package."""

import os
import unittest
import pydocstyle


def get_python_files():
    """Get all python files in the package."""
    pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

    for root, _, files in os.walk(pkg_dir):
        for file in files:
            if file.endswith('.py') and \
                  file != 'setup.py' and file != '__init__.py':
                yield os.path.join(root, file)


class TestPEP257(unittest.TestCase):
    """Class fo testing pep-257 compliance."""

    def test_pep257(self):
        """Test pep-257 compliance."""
        failures = [str(fail) for fail in pydocstyle.check(get_python_files())]

        if failures:
            self.fail(f'Found {len(failures)} ' +
                      'pep-257 failure(s):\n' + '\n'.join(failures))


if __name__ == "__main__":
    unittest.main()
