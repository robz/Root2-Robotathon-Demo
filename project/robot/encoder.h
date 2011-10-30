typedef enum { ENCODER_0, ENCODER_1 } encoder_t;	// encoder data type
typedef signed long encoder_count_t;				// encoder count type

void initEncoders(int backwards1, int backwards2);
signed long getEncoderValue(encoder_t enc);