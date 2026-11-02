"""Moved from host/terminal/views/show_orientation.py: silent_part - nothing in the tree named it."""


def silent_part(record):
    """What to draw when the IMU has produced nothing at all."""
    return [
        '  The IMU has reported %d rotation vectors.' % record['updates'],
        '',
        '  loop %s, %d errors, last error %s.'
        % (record['loop'], record['errors'], record['error']),
        '',
        '  Nothing is drawn because there is nothing to draw: the model at',
        '  identity is a board lying exactly level, which is a plausible',
        '  attitude and would look like a part that works.',
        '',
        '  imu.pins() and imu.product_id() need the loop HELD - call them',
        '  inside board.imu.configuring(), or they answer SERVER DEVICE',
        '  FAILURE and look like a dead part. See FINDINGS.',
    ]
