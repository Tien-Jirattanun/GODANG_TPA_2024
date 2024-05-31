class PIDController:
    def __init__(self, kP, kI, kD, limit=100):
        self.kP = kP
        self.kI = kI
        self.kD = kD
        self.limit = limit
        self.P = 0
        self.I = 0
        self.D = 0
        self.previous_error = 0
        self.output = 0

    def update(self, error):
        self.P = error
        self.I += error
        self.D = error - self.previous_error

        # Reset or reduce the integral term within a certain error threshold
        if abs(error) < 0.05:  # Example threshold, adjust based on your system
            self.I *= 0.5  # Dampen rather than reset to handle very small oscillations

        if self.I > self.limit:
            self.I = self.limit
        elif self.I < -self.limit:
            self.I = -self.limit

        self.output = round((self.kP * self.P) + (self.kI * self.I) + (self.kD * self.D), 2)

        if self.output < -self.limit:
            self.output = -self.limit
        elif self.output > self.limit:
            self.output = self.limit

        self.previous_error = error

        return self.output