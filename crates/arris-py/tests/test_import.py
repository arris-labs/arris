import arris


def test_the_module_imports_with_its_version():
    assert arris.__version__.count(".") >= 2


def test_arris_error_is_an_exception():
    assert issubclass(arris.ArrisError, Exception)
