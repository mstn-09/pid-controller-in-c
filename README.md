# pid-library
This source code is under MIT License

The library files consists of libpid.c and libpid.h files. 
The libpid files consists implementation of pid controller algorithm in c programing language.
It consists of derivative filter, antiwind up schemes, and forwrd,backward Euler and trapizoidal 
method of continous to digital conversion. or (s->z) mapping. 

All the rest files test.c is example how to use the functions. It is not necessary to model
your plant in main function as done in test.c. The response1.bmp, response2.bmp, response3.bmp
shows the ac and step response of plant with pid controller plot by gnuplot in terminal window.


