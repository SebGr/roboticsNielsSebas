"""This tutorial introduces the LeNet5 neural network architecture
using Theano.  LeNet5 is a convolutional neural network, good for
classifying images. This tutorial shows how to build the architecture,
and comes with all the hyper-parameters you need to reproduce the
paper's MNIST results.


This implementation simplifies the model in the following ways:

 - LeNetConvPool doesn't implement location-specific gain and bias parameters
 - LeNetConvPool doesn't implement pooling by average, it implements pooling
   by max.
 - Digit classification is implemented with a logistic regression rather than
   an RBF network
 - LeNet5 was not fully-connected convolutions at second layer

References:
 - Y. LeCun, L. Bottou, Y. Bengio and P. Haffner:
   Gradient-Based Learning Applied to Document
   Recognition, Proceedings of the IEEE, 86(11):2278-2324, November 1998.
   http://yann.lecun.com/exdb/publis/pdf/lecun-98.pdf

"""
import os
import sys
import timeit

import numpy
from numpy import sqrt, prod, ones, floor, repeat, pi, exp, zeros, sum

import theano
import theano.tensor as T
from theano import printing, shared, _asarray, config
from theano.tensor.signal import downsample
from theano.tensor.nnet import conv
from theano.tensor.nnet import neighbours
from theano.tensor.nnet import conv2d

import matplotlib.pyplot as plt
import matplotlib.cm as cm

from logistic_sgd import LogisticRegression, load_data
from mlp import HiddenLayer
floatX = config.floatX


class LeNetConvPoolLayer(object):

    def lcn_pixel(this,neib):
        return (neib-T.mean(neib))/T.std(neib)

    def gaussian_filter(self, kernel_shape):

        x = zeros((kernel_shape, kernel_shape), dtype='float32')

        def gauss(x, y, sigma=2.0):
            Z = 2 * pi * sigma**2
            return  1./Z * exp(-(x**2 + y**2) / (2. * sigma**2))

        mid = floor(kernel_shape/ 2.)
        for i in xrange(0,kernel_shape):
            for j in xrange(0,kernel_shape):
                x[i,j] = gauss(i-mid, j-mid)

        return x / sum(x)

    def lecun_lcn(self, X, kernel_size=5, threshold = 1e-4, use_divisor=False):
        """
        Yann LeCun's local contrast normalization
        Orginal code in Theano by: Guillaume Desjardins
        """

        filter_shape = (1, 1, kernel_size, kernel_size)
        filters = self.gaussian_filter(kernel_size).reshape(filter_shape)
        filters = shared(_asarray(filters, dtype=floatX), borrow=True)

        convout = conv2d(X, filters=filters, filter_shape=filter_shape, 
                            border_mode='full')

        # For each pixel, remove mean of kernel_sizexkernel_size neighborhood
        mid = int(floor(kernel_size/2.))
        new_X = X - convout[:,:,mid:-mid,mid:-mid]

        if use_divisor:
            # Scale down norm of kernel_sizexkernel_size patch
            sum_sqr_XX = conv2d(T.sqr(T.abs_(new_X)), filters=filters, 
                                filter_shape=filter_shape, border_mode='full')

            denom = T.sqrt(sum_sqr_XX[:,:,mid:-mid,mid:-mid])
            per_img_mean = denom.mean(axis=[2,3])
            divisor = T.largest(per_img_mean.dimshuffle(0,1,'x','x'), denom)
            divisor = T.maximum(divisor, threshold)

            new_X /= divisor

        return new_X#T.cast(new_X, floatX)

    def lcnLayer(self, input, **kwargs):
        input_shape = input.shape
        # input_shape_4D = (input_shape[0]*input_shape[1], 1,
        #                     input_shape[2], input_shape[3])
        # input_4D = input.reshape(input_shape_4D, ndim=4)
        out = self.lecun_lcn(input, **kwargs)
        output = out.reshape(input_shape)
        return output


    """Pool Layer of a convolutional network """

    def __init__(self, rng, input, filter_shape, image_shape, poolsize=(2, 2)):
        """
        Allocate a LeNetConvPoolLayer with shared variable internal parameters.

        :type rng: numpy.random.RandomState
        :param rng: a random number generator used to initialize weights

        :type input: theano.tensor.dtensor4
        :param input: symbolic image tensor, of shape image_shape

        :type filter_shape: tuple or list of length 4
        :param filter_shape: (number of filters, num input feature maps,
                              filter height, filter width)

        :type image_shape: tuple or list of length 4
        :param image_shape: (batch size, num input feature maps,
                             image height, image width)

        :type poolsize: tuple or list of length 2
        :param poolsize: the downsampling (pooling) factor (#rows, #cols)
        """

        assert image_shape[1] == filter_shape[1]
        self.input = input

        # there are "num input feature maps * filter height * filter width"
        # inputs to each hidden unit
        fan_in = numpy.prod(filter_shape[1:])
        # each unit in the lower layer receives a gradient from:
        # "num output feature maps * filter height * filter width" /
        #   pooling size
        fan_out = (filter_shape[0] * numpy.prod(filter_shape[2:]) /
                   numpy.prod(poolsize))
        # initialize weights with random weights
        W_bound = numpy.sqrt(6. / (fan_in + fan_out))
        self.W = theano.shared(
            numpy.asarray(
                rng.uniform(low=-W_bound, high=W_bound, size=filter_shape),
                dtype=theano.config.floatX
            ),
            borrow=True
        )

        # the bias is a 1D tensor -- one bias per output feature map
        b_values = numpy.zeros((filter_shape[0],), dtype=theano.config.floatX)
        self.b = theano.shared(value=b_values, borrow=True)

        # convolve input feature maps with filters
        conv_out = conv.conv2d(
            input=input,
            filters=self.W,
            filter_shape=filter_shape,
            image_shape=image_shape
        )

        # # get neighbourhoods for lcn
        # neib_sets = neighbours.images2neibs(
        #     conv_out,
        #     neib_shape=(5,5)
        # )

        # #local contrast normalization
        # lcn_sets, updates = theano.map(fn=self.lcn_pixel,sequences=neib_sets) 

        # #transform back to images
        # lcn_out = neighbours.neibs2images(
        #     lcn_sets,
        #     neib_shape=(5,5),
        #     original_shape=conv_out.shape		
        # )
        lcn_out = self.lcnLayer(conv_out)

        # downsample each feature map individually, using maxpooling
        pooled_out = downsample.max_pool_2d(
            input=lcn_out,
            ds=poolsize,
            ignore_border=True
        )

        # add the bias term. Since the bias is a vector (1D array), we first
        # reshape it to a tensor of shape (1, n_filters, 1, 1). Each bias will
        # thus be broadcasted across mini-batches and feature map
        # width & height
        relu = lambda x: x * (x > 0)
        self.output = relu(pooled_out + self.b.dimshuffle('x', 0, 'x', 'x'))

        # store parameters of this layer
        self.params = [self.W, self.b]

        # keep track of model input
        self.input = input

def evaluate_lenet5(learning_rate=0.01, n_epochs=200,
                    dataset='cifar-10-batches-py',
                    nkerns=[32, 64], batch_size=500):
    """ Demonstrates lenet on MNIST dataset

    :type learning_rate: float
    :param learning_rate: learning rate used (factor for the stochastic
                          gradient)

    :type n_epochs: int
    :param n_epochs: maximal number of epochs to run the optimizer

    :type dataset: string
    :param dataset: path to the dataset used for training /testing (MNIST here)

    :type nkerns: list of ints
    :param nkerns: number of kernels on each layer
    """
    rng = numpy.random.RandomState()


    all_errors = numpy.array([0.0000]*n_epochs*3).reshape(3,n_epochs)

    datasets = load_data(dataset)

    train_set_x, train_set_y = datasets[0]
    valid_set_x, valid_set_y = datasets[1]
    test_set_x, test_set_y = datasets[2]

    # compute number of minibatches for training, validation and testing
    n_train_batches = train_set_x.get_value(borrow=True).shape[0]
    n_valid_batches = valid_set_x.get_value(borrow=True).shape[0]
    n_test_batches = test_set_x.get_value(borrow=True).shape[0]
    n_train_batches /= batch_size
    n_valid_batches /= batch_size
    n_test_batches /= batch_size

    # allocate symbolic variables for the data
    index = T.lscalar()  # index to a [mini]batch

    # start-snippet-1
    x = T.matrix('x')   # the data is presented as rasterized images
    y = T.ivector('y')  # the labels are presented as 1D vector of
                        # [int] labels

    ######################
    # BUILD ACTUAL MODEL #
    ######################
    print '... building the model'

    # Reshape matrix of rasterized images of shape (batch_size, 28 * 28)
    # to a 4D tensor, compatible with our LeNetConvPoolLayer
    # (28, 28) is the size of MNIST images.
    layer0_input = x.reshape((batch_size, 3, 32, 32))

    # Construct the first convolutional pooling layer:
    # filtering reduces the image size to (28-5+1 , 28-5+1) = (24, 24)
    # maxpooling reduces this further to (24/2, 24/2) = (12, 12)
    # 4D output tensor is thus of shape (batch_size, nkerns[0], 12, 12)
    layer0 = LeNetConvPoolLayer(
        rng,
        input=layer0_input,
        image_shape=(batch_size, 3, 32, 32),
        filter_shape=(nkerns[0], 3, 5, 5),
        poolsize=(2, 2)
    )

    # Construct the second convolutional pooling layer
    # filtering reduces the image size to (12-5+1, 12-5+1) = (8, 8)
    # maxpooling reduces this further to (8/2, 8/2) = (4, 4)
    # 4D output tensor is thus of shape (batch_size, nkerns[1], 4, 4)
    layer1 = LeNetConvPoolLayer(
        rng,
        input=layer0.output,
        image_shape=(batch_size, nkerns[0], 14, 14),
        filter_shape=(nkerns[1], nkerns[0], 5, 5),
        poolsize=(2, 2)
    )

    # the HiddenLayer being fully-connected, it operates on 2D matrices of
    # shape (batch_size, num_pixels) (i.e matrix of rasterized images).
    # This will generate a matrix of shape (batch_size, nkerns[1] * 4 * 4),
    # or (500, 50 * 4 * 4) = (500, 800) with the default values.
    layer2_input = layer1.output.flatten(2)

    # construct a fully-connected ReLU layer
    layer2 = HiddenLayer(
        rng,
        input=layer2_input,
        n_in=nkerns[1] * 5 * 5,
        n_out=1250,
        activation=lambda x: x * (x > 0)
    )

    layer3 = HiddenLayer(
        rng,
        input=layer2.output,
        n_in=1250,
        n_out=128,
        activation=lambda x: x * (x > 0)
    )

    # classify the values of the fully-connected ReLU layer
    layer4 = LogisticRegression(input=layer3.output, n_in=128, n_out=10)

    # the cost we minimize during training is the NLL of the model
    cost = layer4.negative_log_likelihood(y)

    # create a function to compute the mistakes that are made by the model
    test_model = theano.function(
        [index],
        layer4.errors(y),
        givens={
            x: test_set_x[index * batch_size: (index + 1) * batch_size],
            y: test_set_y[index * batch_size: (index + 1) * batch_size]
        }
    )

    validate_model = theano.function(
        [index],
        layer4.errors(y),
        givens={
            x: valid_set_x[index * batch_size: (index + 1) * batch_size],
            y: valid_set_y[index * batch_size: (index + 1) * batch_size]
        }
    )

    train_model_errors = theano.function(
        [index],
        layer4.errors(y),
        givens={
            x: train_set_x[index * batch_size: (index + 1) * batch_size],
            y: train_set_y[index * batch_size: (index + 1) * batch_size]
        }
    )

    # create a list of all model parameters to be fit by gradient descent
    params = layer4.params+ layer3.params + layer2.params + layer1.params + layer0.params

    # create a list of gradients for all model parameters
    grads = T.grad(cost, params)

    # train_model is a function that updates the model parameters by
    # SGD Since this model has many parameters, it would be tedious to
    # manually create an update rule for each model parameter. We thus
    # create the updates list by automatically looping over all
    # (params[i], grads[i]) pairs.
    updates = [
        (param_i, param_i - learning_rate * grad_i)
        for param_i, grad_i in zip(params, grads)
    ]

    train_model = theano.function(
        [index],
        cost,
        updates=updates,
        givens={
            x: train_set_x[index * batch_size: (index + 1) * batch_size],
            y: train_set_y[index * batch_size: (index + 1) * batch_size]
        }
    )
    # end-snippet-1

    ###############
    # TRAIN MODEL #
    ###############
    print '... training'
    # early-stopping parameters
    patience = 10000  # look as this many examples regardless
    patience_increase = 2  # wait this much longer when a new best is
                           # found
    improvement_threshold = 0.995  # a relative improvement of this much is
                                   # considered significant
    validation_frequency = min(n_train_batches, patience / 2)
                                  # go through this many
                                  # minibatche before checking the network
                                  # on the validation set; in this case we
                                  # check every epoch

    best_validation_loss = numpy.inf
    best_iter = 0
    test_score = 0.
    start_time = timeit.default_timer()

    epoch = 0
    done_looping = False

    while (epoch < n_epochs) and (not done_looping):
        epoch = epoch + 1

        for minibatch_index in xrange(n_train_batches):

            iter = (epoch - 1) * n_train_batches + minibatch_index

            if iter % 100 == 0:
                print 'training @ iter = ', iter
            cost_ij = train_model(minibatch_index)

            if (iter + 1) % validation_frequency == 0:
                training_losses = [train_model_errors(i) for i
                                     in xrange(n_train_batches)]
                trainingError = numpy.mean(training_losses)

                # compute zero-one loss on validation set
                validation_losses = [validate_model(i) for i
                                     in xrange(n_valid_batches)]
                this_validation_loss = numpy.mean(validation_losses)
                print('epoch %i, minibatch %i/%i, validation error %f %%' %
                      (epoch, minibatch_index + 1, n_train_batches,
                       this_validation_loss * 100.))
                print('training error %f %%' % (trainingError*100))

                test_losses = [
                        test_model(i)
                        for i in xrange(n_test_batches)
                    ]
                test_score = numpy.mean(test_losses)

                all_errors[0,epoch-1]=trainingError
                all_errors[1,epoch-1]=this_validation_loss
                all_errors[2,epoch-1]=test_score

                # if we got the best validation score until now
                if this_validation_loss < best_validation_loss:
                    #print layer0.W.get_value()
                    #plt.figure()
                    plt.ion()
                    for i in xrange(nkerns[0]):
                        plt.subplot(12,8,i+1);
                        plt.imshow(layer0.params[0].get_value()[i][0])
                    for i in xrange(nkerns[1]):
                        plt.subplot(12,8,i+1+nkerns[0]);
                        plt.imshow(layer1.params[0].get_value()[i][0])
                    plt.draw()
                    #improve patience if loss improvement is good enough
                    if this_validation_loss < best_validation_loss *  \
                       improvement_threshold:
                        patience = max(patience, iter * patience_increase)

                    # save best validation score and iteration number
                    best_validation_loss = this_validation_loss
                    best_iter = iter

                    # test it on the test set
                    test_losses = [
                        test_model(i)
                        for i in xrange(n_test_batches)
                    ]
                    test_score = numpy.mean(test_losses)
                    print(('     epoch %i, minibatch %i/%i, test error of '
                           'best model %f %%') %
                          (epoch, minibatch_index + 1, n_train_batches,
                           test_score * 100.))

            if patience <= iter:
                done_looping = True
                break

    numpy.savetxt('trainingValidTest.txt',all_errors)
    end_time = timeit.default_timer()
    print('Optimization complete.')
    plt.show(block=True)
    print('Best validation score of %f %% obtained at iteration %i, '
          'with test performance %f %%' %
          (best_validation_loss * 100., best_iter + 1, test_score * 100.))
    print >> sys.stderr, ('The code for file ' +
                          os.path.split(__file__)[1] +
                          ' ran for %.2fm' % ((end_time - start_time) / 60.))

if __name__ == '__main__':
    evaluate_lenet5()


def experiment(state, channel):
    evaluate_lenet5(state.learning_rate, dataset=state.dataset)
