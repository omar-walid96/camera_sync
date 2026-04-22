#!/usr/bin/env python3
"""Quick capture test for a V4L2 device. Shows a live window and prints FPS."""
import argparse
import time
import cv2

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('device', nargs='?', default='/dev/video8',
                        help='V4L2 device path (default: /dev/video8)')
    args = parser.parse_args()

    cap = cv2.VideoCapture(args.device, cv2.CAP_V4L2)
    if not cap.isOpened():
        print(f"ERROR: could not open {args.device}")
        return 1

    # Report what the driver negotiated
    w   = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h   = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = cap.get(cv2.CAP_PROP_FPS)
    fmt = int(cap.get(cv2.CAP_PROP_FOURCC))
    fourcc = ''.join(chr((fmt >> (8*i)) & 0xFF) for i in range(4))
    print(f"Opened {args.device}  {w}x{h} @ {fps:.1f} fps  fourcc={fourcc}")

    cv2.namedWindow('test_uvc', cv2.WINDOW_AUTOSIZE)

    t0 = time.monotonic()
    frames = 0
    while True:
        ret, frame = cap.read()
        if not ret:
            print("WARNING: read() returned False — no frame")
            time.sleep(0.1)
            continue

        frames += 1
        elapsed = time.monotonic() - t0
        measured_fps = frames / elapsed if elapsed > 0 else 0

        cv2.putText(frame, f"{args.device}  {measured_fps:.1f} fps",
                    (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
        cv2.putText(frame, f"{args.device}  {measured_fps:.1f} fps",
                    (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

        cv2.imshow('test_uvc', frame)
        if cv2.waitKey(1) == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    print(f"Done. Captured {frames} frames in {elapsed:.1f}s  ({frames/elapsed:.1f} fps)")

if __name__ == '__main__':
    main()
