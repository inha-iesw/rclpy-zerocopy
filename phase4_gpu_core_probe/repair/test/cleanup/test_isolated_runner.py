"""A mistyped CTest filter must fail even when gtest emits a valid empty XML."""

import os
import subprocess
import xml.etree.ElementTree as ET

import pytest


@pytest.mark.parametrize('kind', ['PRODUCER', 'SUBSCRIBER'])
def test_missing_isolated_case_fails(kind, tmp_path):
    output = tmp_path / 'empty.xml'
    result = subprocess.run(
        [os.environ[f'G3_{kind}_CLEANUP'], '--gtest_filter=NoSuchSuite.NoSuchCase',
         f'--gtest_output=xml:{output}'], capture_output=True, text=True, timeout=10)
    assert ET.parse(output).getroot().attrib['tests'] == '0'
    assert result.returncode == 2, result.stdout + result.stderr
    assert 'Expected exactly one isolated test; selected 0' in result.stderr
