ASS = thor_hammer
LATEX = pdflatex
ADD = 
BBL=
ALL =  $(ASS)
DST = 


$(ASS).pdf: $(ASS).tex $(BBL) $(ADD)
	pdflatex $(ASS).tex
	pdflatex $(ASS).tex

all: $(ALL)

$(ASS).bbl : $(ASS).bib
	$(LATEX) $(ASS).tex
	bibtex $(ASS)
	$(LATEX) $(ASS).tex
	$(LATEX) $(ASS).tex

open: $(ASS).pdf
	open $(ASS).pdf

html:
	latex2html -no_navigation -info 0 -split 0 $(ASS).tex

spell:
	ispell $(ASS).tex $(ADD)

bib:
	rm -f $(ASS).bbl
	make

force:
	rm -f $(ASS).pdf
	make

upload: $(ASS).pdf
	scp $(ASS).pdf bluenose.cs.dal.ca:public_other/.

tar:
	rm -f $(ASS).tar $(ASS).tar.gz
	tar cvhf $(ASS).tar *
	gzip -9 $(ASS).tar
	mv $(ASS).tar.gz $(DST)
	chmod 644 $(DST)/$(ASS).tar.gz

clean:
	rm -f *.aux *.log *.dvi *.bbl *.blg *.bak $(ASS).ps $(ASS).pdf
