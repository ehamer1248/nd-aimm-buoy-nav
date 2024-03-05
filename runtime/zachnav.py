cubic   = lambda x : x**3
linear  = lambda x : x

def adjust_position(buoy_position, buoy_color):
    # left screen is 0, right 1
    # TODO standardize buoy_position to be in range [0,1]

    # tolerance of 0.3 means buoy must be outside 60% center
    LEFT_TOL    = 0.3
    RIGHT_TOL   = 0.7

    if buoy_position.x < LEFT_TOL and buoy_color == 'green':
        # go left

    elif buoy_position.x > RIGHT_TOL and buoy_color == 'red':
        # go right
