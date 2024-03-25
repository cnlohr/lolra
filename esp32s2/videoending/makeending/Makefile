IMAGEOUT:=../main/image.h

all : $(IMAGEOUT)

$(IMAGEOUT) : image.png imagegen
	./imagegen $< > $@

imagegen : imagegen.c
	gcc -o $@ $^ -Os -lm

clean :
	rm -rf *.o imagegen $(IMAGEOUT)
