import cv2
import numpy as np


def write_video(num_images, sample_num):
    fourcc = cv2.VideoWriter_fourcc(*'DIVX')
    out_done = False
    for i in range(0, num_images):
        if i % sample_num == 0:
            frame = cv2.imread('./media/frame'+str(i).zfill(5)+'.png')
            if frame is not None:
                print(i, './media/frame'+str(i)+'.png', frame.shape)
                frame = cv2.resize(frame, (512, 528), interpolation = cv2.INTER_AREA)
                if not out_done:
                    out = cv2.VideoWriter("./media/astar_nonholonomic_video_3.mp4", fourcc, 1.0, (frame.shape[1], frame.shape[0]))
                    out_done = True
                out.write(frame)

    frame = cv2.imread('./media/frame.png')
    if frame is not None:
        frame = cv2.resize(frame, (512, 528), interpolation = cv2.INTER_AREA)
        out.write(frame)
    out.release()


if __name__ == '__main__':
    write_video(14, 1)

