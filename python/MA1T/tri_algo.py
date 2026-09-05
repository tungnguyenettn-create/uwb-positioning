import numpy as np

def ls(points, distances): 
    points = np.array(points)
    distances = np.array(distances)
    
    # 1. Translate the entire system so the first anchor is at (0,0)
    p0 = points[0]
    shifted_points = points - p0
    
    # 2. Extract the variables for the math
    p_rest = shifted_points[1:]
    r_rest = distances[1:]
    r0_sq = distances[0]**2
    
    # 3. Build the A matrix and b vector using the shifted points
    A = 2 * p_rest 
    b = np.sum(p_rest**2, axis=1) - r_rest**2 + r0_sq
    
    # 4. Matrix Math: Solve for the shifted position
    # PRO TIP: np.linalg.lstsq is mathematically safer than np.linalg.inv
    # because it won't crash if the anchors are in a straight line (singular matrix).
    X_shifted, residuals, rank, s = np.linalg.lstsq(A, b, rcond=None)
    
    # 5. Shift the final calculated position back to real-world coordinates
    X = X_shifted + p0 
    
    return X 




def chan_two_stage(points, distances):
    points = np.array(points)
    distances = np.array(distances)
    
    # ==========================================
    # STEP 1: Pseudo-Linearization 
    # Solve for [x, y, R] assuming R is independent
    # ==========================================
    
    # 1. Build A1 matrix: [-2x, -2y, 1]
    # We create a column of 1s and stack it horizontally next to the -2*points
    ones_col = np.ones((len(points), 1))
    A1 = np.hstack((-2 * points, ones_col))
    
    # 2. Build b1 vector: r^2 - (x^2 + y^2)
    K = np.sum(points**2, axis=1) # Calculates (x^2 + y^2) for every anchor instantly
    b1 = distances**2 - K
    
    # 3. Solve Stage 1
    Z1, _, _, _ = np.linalg.lstsq(A1, b1, rcond=None)
    
    # Extract our rough estimates
    x_est = Z1[0]
    y_est = Z1[1]
    R_est = Z1[2]
    
    # ==========================================
    # STEP 2: Constraint Enforcement
    # Force the math to acknowledge that R = x^2 + y^2
    # ==========================================
    
    # 4. Build Stage 2 equations
    # We are now solving a new matrix to find actual [x^2, y^2]
    A2 = np.array([
        [1, 0],  # x^2 + 0   = x_est^2
        [0, 1],  # 0   + y^2 = y_est^2
        [1, 1]   # x^2 + y^2 = R_est
    ])
    
    b2 = np.array([x_est**2, y_est**2, R_est])
    
    # 5. Solve Stage 2
    Z2, _, _, _ = np.linalg.lstsq(A2, b2, rcond=None)
    x_sq, y_sq = Z2
    
    # 6. Final Coordinate Recovery
    # We take the square root, but we MUST re-apply the positive/negative signs 
    # that we learned from our rough estimate in Step 1.
    # (np.abs protects against math crashes if noise makes the squares slightly negative)
    x_final = np.sign(x_est) * np.sqrt(np.abs(x_sq))
    y_final = np.sign(y_est) * np.sqrt(np.abs(y_sq))
    
    return np.array([x_final, y_final])

def true_chan(points, distances):
    points = np.array(points)
    distances = np.array(distances)
    
    # ==========================================
    # STAGE 1: Weighted Pseudo-Linearization
    # ==========================================
    ones_col = np.ones((len(points), 1))
    A1 = np.hstack((-2 * points, ones_col))
    
    K = np.sum(points**2, axis=1)
    b1 = distances**2 - K
    
    # 1. Create W1 (Weighting Matrix 1)
    # Weights are inversely proportional to the squared distances.
    # W1 = diag(1 / r_i^2)
    W1 = np.diag(1.0 / (distances**2))
    
    # 2. Solve WLS for Stage 1
    # Formula: X = (A^T * W * A)^-1 * A^T * W * b
    A1_T_W1 = A1.T @ W1
    
    # We save this inverted matrix because it is mathematically equivalent 
    # to the Covariance Matrix (the statistical error) of Stage 1!
    Cov1 = np.linalg.inv(A1_T_W1 @ A1) 
    
    Z1 = Cov1 @ A1_T_W1 @ b1
    
    x_est, y_est, R_est = Z1
    
    # ==========================================
    # STAGE 2: Covariance-Weighted Constraint
    # ==========================================
    A2 = np.array([
        [1, 0],
        [0, 1],
        [1, 1]
    ])
    b2 = np.array([x_est**2, y_est**2, R_est])
    
    # 3. Create W2 (Weighting Matrix 2)
    # We build a Jacobian matrix (B2) using our rough estimates
    B2 = np.diag([2 * x_est, 2 * y_est, 1])
    
    # W2 is the inverse of (B2 * Cov1 * B2)
    # This precisely maps the statistical error from Stage 1 into Stage 2
    W2 = np.linalg.inv(B2 @ Cov1 @ B2)
    
    # 4. Solve WLS for Stage 2
    A2_T_W2 = A2.T @ W2
    Z2 = np.linalg.inv(A2_T_W2 @ A2) @ A2_T_W2 @ b2
    
    x_sq, y_sq = Z2
    
    # ==========================================
    # Final Coordinate Recovery
    # ==========================================
    x_final = np.sign(x_est) * np.sqrt(np.abs(x_sq))
    y_final = np.sign(y_est) * np.sqrt(np.abs(y_sq))
    
    return np.array([x_final, y_final]) 


def taylor_series(points, distances, initial_guess, max_iter=10, tol=1e-6):
    points = np.array(points)
    distances = np.array(distances)
    
    # We must start with a decent guess (usually the output of Chan's algorithm)
    guess = np.array(initial_guess, dtype=float)
    
    for iteration in range(max_iter):
        x, y = guess
        
        # 1. Calculate the estimated distances from current guess to all anchors
        dx = x - points[:, 0]
        dy = y - points[:, 1]
        
        # Add a tiny epsilon (1e-10) to prevent division-by-zero if the 
        # guess lands exactly on top of a physical anchor.
        est_distances = np.sqrt(dx**2 + dy**2) + 1e-10 
        
        # 2. Calculate the residual error (Measured - Estimated)
        error = distances - est_distances
        
        # 3. Build the Jacobian Matrix (J)
        # These are the partial derivatives of the distance formula:
        # df/dx = (x - x_i) / est_distance
        # df/dy = (y - y_i) / est_distance
        J = np.column_stack((dx / est_distances, dy / est_distances))
        
        # 4. Solve the Linear Least Squares problem to find the required step
        # Mathematically: delta = (J^T * J)^-1 * J^T * error
        delta, _, _, _ = np.linalg.lstsq(J, error, rcond=None)
        
        # 5. Update the current position guess
        guess += delta
        
        # 6. Convergence Check
        # If the delta step is smaller than our tolerance (e.g., 1 millimeter),
        # we have reached the bottom of the error valley and can stop.
        if np.linalg.norm(delta) < tol:
            break
            
    return guess