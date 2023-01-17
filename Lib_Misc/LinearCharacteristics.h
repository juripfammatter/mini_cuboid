// Linear Characteristics for different purposes (map Voltage to acc etc.)


#ifndef LINEAR_CHARACTERISTICS_H_
#define LINEAR_CHARACTERISTICS_H_   


class LinearCharacteristics{
     public:
            LinearCharacteristics(){};
            LinearCharacteristics(float, float);
            float evaluate(float);
            float operator()(float x){
                return evaluate(x);
                } 
            virtual     ~LinearCharacteristics();
                
    private:
        // here: private functions and values...
        float gain;
        float offset;
        float ulim;
        float llim;
};


#endif      // LINEAR_CHARACTERISTICS_H_
