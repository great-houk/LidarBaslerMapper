# Makefile for Basler pylon sample program
.PHONY: all clean

# The program to build
NAME       := main

# Target dir
TARGET := target

# Installation directories for pylon
PYLON_ROOT ?= /opt/pylon

# Build tools and flags
LD         := $(CXX)
CPPFLAGS   := $(shell $(PYLON_ROOT)/bin/pylon-config --cflags)
CXXFLAGS   := #e.g., CXXFLAGS=-g -O0 for debugging
LDFLAGS    := $(shell $(PYLON_ROOT)/bin/pylon-config --libs-rpath)
LDLIBS     := $(shell $(PYLON_ROOT)/bin/pylon-config --libs)

# Rules for building
all: clean $(NAME)

$(NAME): $(NAME).o
	$(LD) $(LDFLAGS) -o $(TARGET)/$@ $(TARGET)/$^ $(LDLIBS)

$(NAME).o:
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) src/$(NAME).cpp -c -o $(TARGET)/$@ $<

clean:
	$(RM) $(TARGET)/$(NAME).o $(TARGET)/$(NAME)