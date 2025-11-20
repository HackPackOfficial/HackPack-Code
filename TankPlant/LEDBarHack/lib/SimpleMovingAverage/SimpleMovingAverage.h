class SimpleMovingAverage {
private:
    int *samples;
    int numSamples;
    int currentIndex;
    int sum;

public:
    // Constructor
    SimpleMovingAverage(int numSamples) {
        // Serial.print("d1");
        this->numSamples = numSamples;
        samples = new int[numSamples];
        reset();  // Initialize samples to 0
    }

    // Copy Constructor
    SimpleMovingAverage(const SimpleMovingAverage &other) {
        numSamples = other.numSamples;
        samples = new int[numSamples];
        for (int i = 0; i < numSamples; ++i) {
            samples[i] = other.samples[i];
        }
        currentIndex = other.currentIndex;
        sum = other.sum;
    }

    // Copy Assignment Operator
    SimpleMovingAverage& operator=(const SimpleMovingAverage &other) {
        if (this == &other) {
            return *this; // Handle self-assignment
        }

        // Clean up existing resources
        delete[] samples;

        // Copy from other
        numSamples = other.numSamples;
        samples = new int[numSamples];
        for (int i = 0; i < numSamples; ++i) {
            samples[i] = other.samples[i];
        }
        currentIndex = other.currentIndex;
        sum = other.sum;

        return *this;
    }

    // Destructor
    ~SimpleMovingAverage() {
        delete[] samples;
    }

    // Method to reset all samples to 0
    void reset() {
        // Serial.print("\n\nnumSamples = ");
        // Serial.print(numSamples);
        // Serial.print("\t\t");
        // Serial.println("d2-");
        for (int i = 0; i < numSamples; i++) {
            // Serial.print(i);
            samples[i] = 0;
            delay(2);
        }
        currentIndex = 0;
        sum = 0;
        // Serial.println("\n\nd3");
    }

    // Method to add a new value and get the filtered result
    int filter(int newValue) {
        // Subtract the oldest value from the sum
        sum -= samples[currentIndex];
        // Add the new value to the sum
        sum += newValue;
        // Store the new value in the current index
        samples[currentIndex] = newValue;
        // Move to the next index, wrapping around if necessary
        currentIndex = (currentIndex + 1) % numSamples;
        // Return the average
        return sum / numSamples;
    }
};
