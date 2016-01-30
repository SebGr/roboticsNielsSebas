from __future__ import print_function
import theano
a = theano.tensor.vector()  # declare variable
b = theano.tensor.vector()  # declare variable
out = a ** 2 + b ** 2 + 2 * a * b  # build symbolic expression
f = theano.function([a, b], out)   # compile function
print(f([3, 1], [4, 1]))  # prints [ 49.  4.]