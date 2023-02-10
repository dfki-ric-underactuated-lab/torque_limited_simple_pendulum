SPHINX_APIDOC_OPTIONS=members,undoc-members,show-inheritance sphinx-apidoc -o ../text/generated/ ../../../software/python/simple_pendulum/ -f -M -T
# sphinx-apidoc doesn't allow setting maxdepth on subpackages
sed -i "s/:maxdepth: 4/:maxdepth: 1/g" ../text/generated/*
# sphinx toctree is very indentation sensitive, make it uniform
sed -i "s/    /   /g" ../text/generated/*
sed -i "s/ package//" ../text/generated/*
sed -i "s/ module//" ../text/generated/*
