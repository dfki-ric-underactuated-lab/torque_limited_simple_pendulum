PYTHON_ROOT = "software/python"
DOC_ROOT = "docs/reference/src"

default: install

install:
	make -C $(PYTHON_ROOT) install

mininstall:
	make -C $(PYTHON_ROOT) mininstall

doc: 
	make -C $(DOC_ROOT) clean
	make -C $(DOC_ROOT) generate_rst
	make -C $(DOC_ROOT) html

tests:
	make -C $(PYTHON_ROOT) test
