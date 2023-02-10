# Install dependencies
# sudo apt-get install texlive
# sudo apt-get install texlive-lualatex
# sudo apt-get install texlive-bibtex-extra biber
# ... [your packages -> Search Debian packages for <your package>.sty (https://packages.debian.org/index)]

# Clean
latexmk -c

# Compile
latexmk -bibtex -r latexmkrc main -lualatex
echo '~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~'

# Clean again
latexmk -c
rm main.bbl
rm main.run.xml