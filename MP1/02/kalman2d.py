import sys
import numpy as np
import matplotlib.pyplot as plt

Q = np.array([[0.0001, 0.00002], [0.00002, 0.0001]])
R = np.array([[0.01, 0.005], [0.005, 0.02]])


# X and U and are 2x1 matrixes
# return: x_priori and p_priori
def predict(x, u, p):
    x_priori = np.add(x, u)
    p_priori = np.add(p, Q)

    return x_priori, p_priori


def update(p_priori, x_priori, z):
    k_filter = np.divide(p_priori, np.add(p_priori, R))
    x_estimate = np.add(x_priori, np.matmul(k_filter, np.subtract(z, x_priori)))
    p_estimate = np.multiply(np.subtract(np.eye(2), k_filter), p_priori)

    return x_estimate, p_estimate


if __name__ == "__main__":

    # Retrive file name for input data
    if (len(sys.argv) < 5):
        print("Four arguments required: python kalman2d.py [datafile] [x1] [x2] [lambda]")
        exit()

    filename = sys.argv[1]
    x10 = float(sys.argv[2])
    x20 = float(sys.argv[3])
    scaler = float(sys.argv[4])

    x = np.array([[x10], [x20]])
    p = np.array([[scaler, 0], [0, scaler]])

    # Read data
    lines = [line.rstrip('\n') for line in open(filename)]
    data = []
    for line in range(0, len(lines)):
        data.append(list(map(float, lines[line].split(' '))))

    # Print out the data
    print("The input data points in the format of 'k [u1, u2, z1, z2]', are:")
    resultX1 = []
    resultX2 = []
    resultZ1 = []
    resultZ2 = []
    for it in range(0, len(data)):
        u1 = data[it][0]
        u2 = data[it][1]
        z1 = data[it][2]
        z2 = data[it][3]

        u = np.array([[u1], [u2]])
        z = np.array([[z1], [z2]])

        # get the U and Z
        x_priori, p_priori = predict(x, u, p)
        x, p = update(p_priori, x_priori, z)

        resultX1.append(x[0])
        resultX2.append(x[1])
        resultZ1.append(z[0])
        resultZ2.append(z[1])

        print("X")
        print(x)
        print("Z")
        print(z)
        print("\n\n")

    plt.plot(resultX1, resultX2, '-o', label='Prediction')
    plt.plot(resultZ1, resultZ2, '-ro', label='Observation')
    plt.xlabel("x1/z1")
    plt.ylabel("x2/z2")
    plt.title("Prediction vs Observation")
    plt.legend()
    plt.show()