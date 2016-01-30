data = read.table('trainingValidTest.txt')
data = data*10

range = c(1:200)

plot(range, data[1,], type='l', col="red", ylim=c(0,100), ann=FALSE)
lines(range, data[2,], col="green")
lines(range, data[3,], col="black")
title('Training, validation and test error for a CIFAR dataset with LCN', xlab='Epochs', ylab='Error(in %)')
name_errors = c('Training error', 'Validation error', 'Test error')

legend('topright', name_errors, cex=0.8, col=c('red','green','black'), lty=1);

