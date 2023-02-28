// Linear Characteristics for different purposes (map Voltage to acc etc.)
#pragma once
class LinearCharacteristics{
     public:
            LinearCharacteristics(){};              // default constructor
            LinearCharacteristics(float gain,float offset);    // constructor with gain and offset
            LinearCharacteristics(float xmin,float max, float ymin, float ymax);    // constructor with gain and offset
            float evaluate(float);                  // calculate y(x)
            float operator()(float x){              // calculate with () operator; Functor
                return evaluate(x);
                } 
            virtual     ~LinearCharacteristics();   // deconstructor
                
    private:
        // here: private functions and values...
        float gain;
        float offset;
        float ulim;
        float llim;
};
