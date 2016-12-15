def temp_coeff_hot(lower, upper):
	"""
	Calculates and returns the m and b coefficients for y = m*x + b
	for a line intersecting (lower, 255) and (upper, 0).
	"""
	m = -255/(upper-lower)
	b = 255 - m * lower
	return m, b

def temp_coeff_cold(lower, upper):
	"""
	Calculates and returns the m and b coefficients for y = m*x + b
	for a line intersecting (lower, 0) and (upper, 255).
	"""
	m = 255/(upper-lower)
	b = 255 - m * upper
	return m, b