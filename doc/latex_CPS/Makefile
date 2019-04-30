all:
	pdflatex hare-paper.tex
	bibtex hare-paper
	pdflatex hare-paper.tex
	pdflatex hare-paper.tex
	rm -f *.aux
	rm -f *.toc
	# rm -f *.pdf
	rm -f *.xml
	rm -f *.bcf
	rm -f *.bbl
	rm -f *.blg
	rm -f *.log
	rm -f *.out
	rm -f *~
	rm -f ../hare-paper.pdf
	cp hare-paper.pdf ..
clean:
	rm -f *.aux
	rm -f *.toc
	# rm -f *.pdf
	rm -f *.xml
	rm -f *.bcf
	rm -f *.bbl
	rm -f *.blg
	rm -f *.log
	rm -f *.out
	rm -f *~
