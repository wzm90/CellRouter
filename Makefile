###########################################################################
#
# A Makefile template for programming in c/c++ under CLI
#
# Author: Zimin Wang (simon.zmwang@gmail.com)
#
# Usage:
#   $ make             Compile and link
#   $ make clean       Clean the objectives and target
#   $ make cleanobj    Clean the objectives 
#
###########################################################################

include ./macro.defs

TARGET := main

all_srcs := $(wildcard *.cpp)
all_objs := $(all_srcs:.cpp=.o)
DEP := $(patsubst %.cpp,.%.d,$(all_srcs))

PHONY = all clean cleanobj

all: $(TARGET)

$(TARGET): $(all_objs) $(OA_LIB_LIST)
	$(CCPATH) $(CXXOPTS) -o $@ $^ \
         $(COMMON_CODE) \
	 -L$(OA_LIB_DIR) \
	 -loaCommon \
	 -loaBase \
	 -loaPlugIn\
	 -loaDM\
	 -loaTech\
         -loaDesign\
	 $(SYSLIBS)

$(all_objs): %.o:%.cpp
	$(CCPATH) $(CXXOPTS) $(DEBUG) -I$(TOOLSDIR)/include/oa \
	 -I$(TOOLSDIR)/include \
	 -c $<

# automatic header file dependencies
$(DEP): .%.d:%.cpp
	@set -e; rm -rf $@; \
	$(CCPATH) -I$(TOOLSDIR)/include/oa -I$(TOOLSDIR)/include -MM $< >$@.$$$$; \
	sed 's,\($*\)\.o[ :]*,\1.o $@ : ,g' <$@.$$$$ >$@;\
	rm -rf $@.$$$$;
-include $(DEP)

clean: cleanobj
	rm -rf $(TARGET) $(DEP)

cleanobj:
	rm -rf $(all_objs)
