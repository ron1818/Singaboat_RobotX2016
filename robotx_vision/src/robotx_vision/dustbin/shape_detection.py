# import the necessary packages
import cv2


class ShapeDetector:
    def __init__(self):
        pass

    def detect(self, c):
        # initialize the shape name and approximate the contour
        shape = "unidentified"
        peri = cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, 0.04 * peri, True)
        return approx


if __name__ == "__main__":
    # read image
    img = cv2.imread("image/green_triangle.png", cv2.IMREAD_COLOR)
    # plot, debug only
    # sd = ShapeDetector()
    # sd.detect(img)
    peri = cv2.arcLength(img, True)
    approx = cv2.approxPolyDP(img, 0.04 * peri, True)
    print approx
    cv2.imshow('image', img)
    cv2.moveWindow('image', 250, 350)

    # cv2.waitKey(0)  # wait inf for a keystroke
    k = cv2.waitKey(0)
    if k == 27:  # esc key to exit
        cv2.destroyAllWindows()
    elif k == ord('s'):  # save key to save and exit
        cv2.imwrite('hat2gray.png', img)
        cv2.destroyAllWindows()

