##### LIBRARIES CONFIGURATION #####

SYSTEMC_CANDIDATES := libs/systemc-2.3.3/build/install libs/systemc-2.3.1/objdir/install
SYSTEMC := $(firstword $(foreach d,$(SYSTEMC_CANDIDATES),$(if $(wildcard $(d)/include),$(d))))
YAML    := libs/yaml-cpp/build

ifeq ($(SYSTEMC),)
$(error Could not find a valid SystemC install. Checked: $(SYSTEMC_CANDIDATES))
endif


##### DIRECTORIES #####

SYSTEMC_LIBS := $(firstword $(wildcard $(SYSTEMC)/lib-linux64) $(wildcard $(SYSTEMC)/lib-macosx64) $(wildcard $(SYSTEMC)/lib-macosx) $(wildcard $(SYSTEMC)/lib*))

ifeq ($(SYSTEMC_LIBS),)
$(error Could not find SystemC library directory under $(SYSTEMC))
endif

SRCDIR  := ../src
SUBDIRS := $(filter %/,$(wildcard $(SRCDIR)/*/))
OBJDIR  := ./build

SRCS := $(wildcard $(SRCDIR)/*.cpp) $(wildcard $(addsuffix *.cpp,$(SUBDIRS)))
OBJS := $(subst $(SRCDIR),$(OBJDIR),$(SRCS:.cpp=.o))

MODULE := noxim


##### COMPILER OPTIONS #####

CXX      := g++
OPT      := -O3 -march=native -fno-strict-aliasing
# to enable logs with LOG macro, uncomment "-g DDEBUG" in the line below
# and recompile everythin (make clean)
# DEBUG    :=  -g -DDEBUG
OTHER    := -Wall -DSC_NO_WRITE_CHECK --std=c++17 # Match SystemC ABI (cxx201703L)
CXXFLAGS := $(OPT) $(OTHER) $(DEBUG)

INCDIR := -I$(SRCDIR) -isystem $(SYSTEMC)/include -I$(CURDIR)/libs/yaml-cpp/include
LIBDIR := -L$(SYSTEMC_LIBS) -L$(YAML)

LIBS := -lsystemc -lm -lyaml-cpp 

SPACE := $(subst ,, )
VPATH := $(SRCDIR):$(subst $(SPACE),:,$(SUBDIRS))

##### RULES #####

$(MODULE): $(OBJS)
	$(CXX) $(LIBDIR) $^ $(LIBS) -o $@ 2>&1 | c++filt

$(OBJDIR)/%.o: %.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) $(INCDIR) -c $< -o $@

clean:
	rm -f $(OBJS) $(MODULE) *~

depend: 
	makedepend $(SRCS) -Y -f- 2>/dev/null | sed 's_$(SRCDIR)_$(OBJDIR)_' > Makefile.deps

-include Makefile.deps
