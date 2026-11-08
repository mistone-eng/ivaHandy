"""package.xml schema (ament_xmllint)."""
import pytest
from ament_xmllint.main import main


@pytest.mark.linter
@pytest.mark.xmllint
def test_xmllint():
    rc = main(argv=[])
    assert rc == 0, 'Found errors'
