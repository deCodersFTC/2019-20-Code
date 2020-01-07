@Override
public void onQCARUpdate(State state) {
    Image imageRGB565 = null;
    Frame frame = state.getFrame();

    for (int i = 0; i < frame.getNumImages(); ++i) {
        Image image = frame.getImage(i);
        if (image.getFormat() == PIXEL_FORMAT.RGB565) {
            imageRGB565 = image;
            break;
        }
    }

    if (imageRGB565 != null) {
        ByteBuffer pixels = imageRGB565.getPixels();
        byte[] pixelArray = new byte[pixels.remaining()];
        pixels.get(pixelArray, 0, pixelArray.length());
        int imageWidth = imageRGB565.getWidth();
        int imageHeight = imageRGB565.getHeight();
        int stride = imageRGB565.getStride();

        Options opts = new BitmapFactory.Options();
        opts.inPreferredConfig = Bitmap.Config.RGB;
        Bitmap bm = BitmapFactory.decodeByteArray(pixelArray, 0, pixelArray.length, opts);

        // m_ivCamera is a android.widget.ImageView object.
        m_ivCamera.setImageBitmap(bm);
    }
