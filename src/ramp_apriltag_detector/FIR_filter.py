from collections import deque
import numpy as np
import matplotlib.pyplot as plt
from dataclasses import dataclass, field
from typing import List

@dataclass
class LowPassFilterCoefficients(list):
    """
    Low pass (windowed-sinc) FIR, using a Rectangular window, 
    30Hz sampling rate, 1.5Hz Cutoff frequency, and 1.5Hz transition bandwidth.
    Characteristics:
        95% Rise time, 260ms.
        Peak unit impulse response, 0.171
    """
    coeffs: List[float] = field(default_factory=lambda: [
        0.000000000000000007,
        0.039899882278863805,
        0.086079154232428276,
        0.129118731348642435,
        0.159599529115455163,
        0.170605406049220421,
        0.159599529115455163,
        0.129118731348642435,
        0.086079154232428276,
        0.039899882278863805,
        0.000000000000000007
    ])

    def __post_init__(self):
        # This ensures the 'list' part of the object is populated 
        # with the coefficients for compatibility with FIR logic.
        super().__init__(self.coeffs)

@dataclass
class MediumPassFilterCoefficients(list):
    """
    Low pass (windowed-sinc) FIR, using a Rectangular window, 
    30Hz sampling rate, 1.5Hz Cutoff frequency, and 1.5Hz transition bandwidth.
    Characteristics:
        95% Rise time, 260ms.
        Peak unit impulse response, 0.171
    """
    coeffs: List[float] = field(default_factory=lambda: [
        0.028379725838660386,
        0.271502373147798348,
        0.400235802027082532,
        0.271502373147798348,
        0.028379725838660386,
    ])

    def __post_init__(self):
        # This ensures the 'list' part of the object is populated 
        # with the coefficients for compatibility with FIR logic.
        super().__init__(self.coeffs)


class DiscreteFIRFilter:
    def __init__(self, dim: int, coefficients: list):
        """
        FIR Filter implementation matching the DiscreteManualLTI interface.
        
        :param dim: The number of signals to filter (e.g., 6 for x,y,z,R,P,Y)
        :param coefficients: The list of FIR weights
        """
        self.N = dim
        # Convert coefficients to a numpy array for vectorization
        self.coeffs = np.array(coefficients, dtype=float)
        self.num_taps = len(self.coeffs)
        
        # Interface compatibility: ensure it looks like the LTI version
        # (Though FIR doesn't use these, we keep them to prevent attribute errors)
        self.dt = None 
        
        # We need a buffer for each dimension to store input history
        # Shape: (Dimensions, Number of Taps)
        self.history = np.zeros((self.N, self.num_taps))
        
        # To match the advance output format
        self.y_k1 = np.zeros(self.N)

    def reset(self, x_init=None, u_init=None):
        """Resets history. If u_init is provided, fills the buffer with that value."""
        if u_init is not None:
            # If u_init is a vector of size self.N, fill the history for each
            for i in range(self.N):
                self.history[i, :] = u_init[i]
        else:
            self.history *= 0.0
        
        self.y_k1 *= 0.0

    def initialise(self, iters=None, x_init=None, u_init=None):
        """
        For FIR, initialisation is simpler: we just fill the buffer 
        with the initial value to prevent 'ramp-up' delay.
        """
        self.reset(x_init=x_init, u_init=u_init)

    def advance(self, u_k):
        """
        Shifts history and computes the dot product with coefficients.
        
        u_k: Input vector of size self.N
        """
        # 1. Shift history to the right
        self.history[:, 1:] = self.history[:, :-1]
        
        # 2. Add new input to the start
        self.history[:, 0] = u_k
        
        # 3. Compute dot product: sum(history * coeffs)
        # Using np.tensordot for efficient vectorized calculation across all dimensions
        self.y_k1 = np.tensordot(self.history, self.coeffs, axes=((1), (0)))
        
        return self.y_k1
    


# - Test Script ---

def main():
    # 1. Setup Filter
    fc = LowPassFilterCoefficients()
    dim = 6
    fir = DiscreteFIRFilter(dim=dim, coefficients=fc.coeffs)
    
    # 2. Create Test Signal (Impulse on Channel 0, Step on Channel 1)
    # Total samples to observe full window and tail
    n_samples = 40 
    test_input = np.zeros((n_samples, dim))
    
    # Channel 0: Impulse at index 5
    test_input[5, 0] = 1.0 
    
    # Channel 1: Step starting at index 5
    test_input[5:, 1] = 1.0 
    
    # 3. Process signal
    outputs = np.zeros((n_samples, dim))
    for i in range(n_samples):
        outputs[i] = fir.advance(test_input[i])
        
    # 4. Plotting
    plt.figure(figsize=(10, 6))
    
    # Plot Impulse Response (Channel 0)
    plt.subplot(2, 1, 1)
    plt.stem(outputs[:, 0], linefmt='b-', markerfmt='bo', basefmt='r-', label='Filter Output (Impulse)')
    plt.plot(test_input[:, 0], 'k--', alpha=0.3, label='Input Impulse')
    plt.title("FIR Filter Impulse Response (3Hz Cutoff, 30Hz Sampling)")
    plt.ylabel("Amplitude")
    plt.legend()
    plt.grid(True, alpha=0.3)

    # Plot Step Response (Channel 1)
    plt.subplot(2, 1, 2)
    plt.plot(outputs[:, 1], 'g-', linewidth=2, label='Filter Output (Step)')
    plt.plot(test_input[:, 1], 'k--', alpha=0.3, label='Input Step')
    plt.axhline(1.0, color='r', linestyle=':', label='Steady State Target')
    plt.title("FIR Filter Step Response (Verification of Zero Steady-State Error)")
    plt.xlabel("Sample Index")
    plt.ylabel("Amplitude")
    plt.legend()
    plt.grid(True, alpha=0.3)

    plt.tight_layout()
    print("Plotting complete. Steady state value:", outputs[-1, 1])
    plt.show()

if __name__ == "__main__":
    main()