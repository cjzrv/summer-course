# Define target file and source files
TARGET = test
SRCS = hw1.cpp

# Compiler and its options
CXX = g++
CXXFLAGS = `pkg-config --cflags opencv4`
LDFLAGS = `pkg-config --libs opencv4`

# Rule: Compile the target
$(TARGET): $(SRCS)
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(SRCS) $(LDFLAGS)

# Rule: Clean up generated files
clean:
	rm -f $(TARGET)

run: $(TARGET)
	./$(TARGET)

.PHONY: $(TARGET) clean run