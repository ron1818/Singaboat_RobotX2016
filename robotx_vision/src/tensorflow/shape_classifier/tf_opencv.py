import tensorflow as tf
import cv2

def get_image(image_path):
    """Reads the jpg image from image_path.
    Returns the image as a tf.float32 tensor
    Args:
        image_path: tf.string tensor
    Reuturn:
        the decoded jpeg image casted to float32
    """
    return tf.image.convert_image_dtype(
        tf.image.decode_jpeg(
            tf.read_file(image_path), channels=3),
        dtype=tf.uint8)


path = "test_photos/circle0.jpg"
raw_image_data = cv2.imread(path, 1)

# image = tf.placeholder("uint8", [None, None, 3])
x = tf.Variable(raw_image_data, name='x')
model = tf.initialize_all_variables()
x = tf.transpose(x, perm=[1,0,2])
# slice = tf.slice(image, [0, 0, 0], [50, -1, -1])

with tf.Session() as session:
    # result = session.run(slice, feed_dict={image: raw_image_data})
    session.run(model)
    result = session.run(x)
    # result2 = session.run(x)
    # print(result.shape)

cv2.namedWindow('image', 0)
cv2.imshow('image', result)
cv2.waitKey(0)
