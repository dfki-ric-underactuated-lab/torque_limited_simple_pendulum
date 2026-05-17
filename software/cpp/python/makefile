LIB_DIR = "../controllers/ilqr/obj"
LIB_DIR_ROOT = "../controllers/ilqr"

default: cppilqr

cppilqr : setup.py cilqr.pyx $(LIB_DIR)/libilqr.a
	python3 setup.py build_ext --inplace && rm -f cilqr.cpp && rm -Rf build && bash move_pylib.sh

$(LIB_DIR)/libilqr.a:
	make -C $(LIB_DIR_ROOT) obj/libilqr.a

clean:
	rm *.so
	#rm cilqr.cpp
