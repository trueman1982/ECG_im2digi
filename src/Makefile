OPENCV_CFLAG = `pkg-config --cflags --libs opencv`
INCLUDE = ./

test_digi_src = digi_demo_web.cpp SegmentFactory.cpp Segment.cpp SegmentLine.cpp
test_digi_obj = $(test_digi_src:.cpp=.o)

test_digi : $(test_digi_obj)
	echo $(test_digi_obj)
	g++ $(test_digi_obj) $(OPENCV_CFLAG)  -o digi_demo_web

%.o : %.cpp
	g++ -c $< -I$(INCLUDE) -o $@

clean :
	rm *.o
