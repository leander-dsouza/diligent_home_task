#! /usr/bin/env python3
"""Script to analyse pep8 linting errors in the package."""
import os
import re
import unittest
import pycodestyle


def get_python_files():
    """Get all python files in the package."""
    pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

    for root, _, files in os.walk(pkg_dir):
        for file in files:
            if file.endswith('.py') and \
                  file != 'setup.py' and file != '__init__.py':
                yield os.path.join(root, file)


class StringReport(pycodestyle.StandardReport):
    """Class to handle structuring of the error messages."""

    def get_failures(self):
        """Return the list of failures."""
        err_strings = []
        if self.total_errors > 0:
            self._deferred_print.sort()
            for line_number, offset, code, text, _ in self._deferred_print:
                err_strings.append(self._fmt % {
                    'path': self.filename,
                    'row': self.line_offset + line_number, 'col': offset + 1,
                    'code': code, 'text': text,
                })
                if line_number > len(self.lines):
                    line = ''
                else:
                    line = self.lines[line_number - 1]
                err_strings.append(line.rstrip())
                err_strings.append(re.sub(r'\S', ' ', line[:offset]) + '^')
        return err_strings

    def get_file_results(self):
        """Return the results for a single file."""
        return self.file_errors


class TestPEP8(unittest.TestCase):
    """Class fo testing pep-8 compliance."""

    def test_pep8(self):
        """Test pep-8 compliance."""
        pep8opts = pycodestyle.StyleGuide().options

        report = StringReport(pep8opts)
        failures = []
        for file in get_python_files():
            checker = pycodestyle.Checker(
                file, options=pep8opts, report=report)
            checker.check_all()
            report = checker.report
            if report.get_file_results() > 0:
                failures.extend(report.get_failures())
                failures.append('')

        if failures:
            self.fail(f'Found {report.total_errors} ' +
                      'pep-8 failure(s):\n' + '\n'.join(failures))


if __name__ == "__main__":
    unittest.main()
