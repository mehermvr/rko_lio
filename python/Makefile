.PHONY: install
install:
	pip install -v ".[all]"

.PHONY: editable
editable:
	pip install scikit-build-core pyproject_metadata pathspec pybind11 ninja cmake && pip install --no-build-isolation -ve  ".[all]"
