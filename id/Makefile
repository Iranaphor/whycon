CFLAGS := -ggdb -std=c++14 -Wall -O2 -march=native
CFLAGS += `Magick++-config --cxxflags --cppflags`
LDFLAGS := -lm
LDFLAGS += `Magick++-config --ldflags --libs`

all: whycon_id_gen

whycon_id_gen: whycon_id_gen.cpp CNecklace.cpp
	$(CXX) $(CFLAGS) whycon_id_gen.cpp CNecklace.cpp $(LDFLAGS) -o $@ 

clean:
	rm -f *.o whycon_id_gen

.PHONY: clean
