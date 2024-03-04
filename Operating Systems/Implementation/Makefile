SHELL=/bin/bash

.PHONY: all
all: fcfs sjf srtf rr

fcfs:

sjf:

srtf:

rr:

.PHONY: test
test: test-fcfs test-sjf test-srtf test-rr

.PHONY: test-fcfs
test-fcfs: fcfs
	@for i in `ls tests/INPUT*`; do \
		[ -f tests/fcfs$$( echo $$i | tr -d 'A-z/' ) ] && \
		diff -qB \
		tests/fcfs$$( echo $$i | tr -d 'A-z/' ) \
		<( cat $$i | ./fcfs ) || true; \
	done

.PHONY: test-sjf
test-sjf: sjf 
	@for i in `ls tests/INPUT*`; do \
		[ -f tests/sjf$$( echo $$i | tr -d 'A-z/' ) ] && \
		diff -qB \
		tests/sjf$$( echo $$i | tr -d 'A-z/' ) \
		<( cat $$i | ./sjf ) || true; \
	done

.PHONY: test-srtf
test-srtf: srtf 
	@for i in `ls tests/INPUT*`; do \
		[ -f tests/srtf$$( echo $$i | tr -d 'A-z/' ) ] && \
		diff -qB \
		tests/srtf$$( echo $$i | tr -d 'A-z/' ) \
		<( cat $$i | ./srtf ) || true; \
	done

.PHONY: test-rr
test-rr: rr 
	@for i in `ls tests/INPUT*`; do \
		[ -f tests/rr$$( echo $$i | tr -d 'A-z/' ) ] && \
		diff -qB \
		tests/rr$$( echo $$i | tr -d 'A-z/' ) \
		<( cat $$i | ./rr ) || true; \
	done

.PHONY: clean
clean:
	rm -f fcfs sjf srtf rr scheduling.tar.gz

.PHONY: tar
tar:
	tar czf scheduling.tar.gz fcfs.c sjf.c srtf.c rr.c tests

.PHONY: memory_check
memory_check: fcfs sjf srtf rr
	cat tests/INPUT01 | valgrind -s ./fcfs
	cat tests/INPUT01 | valgrind -s ./sjf
	cat tests/INPUT01 | valgrind -s ./srtf
	cat tests/INPUT01 | valgrind -s ./rr

	cat tests/INPUT02 | valgrind -s ./fcfs
	cat tests/INPUT02 | valgrind -s ./sjf
	cat tests/INPUT02 | valgrind -s ./srtf
	cat tests/INPUT02 | valgrind -s ./rr
