void
drawRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, colour col)
{
    writeCommand(kSSD1331CommandDRAWRECT);

    // start x, y
    writeCommand(x);
    writeCommand(y);

    // end x, y
    writeCommand(x+w-1);
    writeCommand(y+h-1);

    // outline
    writeCommand(col.r);
    writeCommand(col.g);
    writeCommand(col.b);

    // fill
    writeCommand(col.r);
    writeCommand(col.g);
    writeCommand(col.b);
}

void
drawLine(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, colour col)
{
    writeCommand(kSSD1331CommandDRAWLINE);

    // start x, y
    writeCommand(x1);
    writeCommand(y1);

    // end x, y
    writeCommand(x2);
    writeCommand(y2);

    // color
    writeCommand(col.r);
    writeCommand(col.g);
    writeCommand(col.b);
}

fpoint_2d
rasterizePoint(fpoint_3d p)
{
    static const float WIDTH = 96.0f;
    static const float HEIGHT = 64.0f;

    fpoint_2d pt = {
        (uint8_t) (WIDTH  / 2 + p.x / (1 + p.z/ZSCALE) ),
        (uint8_t) (HEIGHT / 2 + p.y / (1 + p.z/ZSCALE) )
    };

    return pt;
}

void
draw3DLine(fpoint_3d start, fpoint_3d end, colour col)
{
    fpoint_2d s = rasterizePoint(start);
    fpoint_2d e = rasterizePoint(end);

    drawLine(s.x, s.y, e.x, e.y, col);
}

void
draw3DRect(float width, float z, colour col)
{
    fpoint_2d origin = rasterizePoint(
        (fpoint_3d) { -width/2, -width/2, z }
    );

    uint8_t start_x = (uint8_t) origin.x;
    uint8_t start_y = (uint8_t) origin.y;
    uint8_t w = (uint8_t) width / (1 + z/ZSCALE);

    drawRect(start_x, start_y, w, w, col);
}

void
drawCube(float width, colour col)
{
    fpoint_3d cube[] = {
        { -width/2, -width/2,      0 },
        {  width/2, -width/2,      0 },
        {  width/2,  width/2,      0 },
        { -width/2,  width/2,      0 },
        { -width/2, -width/2,  width },
        {  width/2, -width/2,  width },
        {  width/2,  width/2,  width },
        { -width/2,  width/2,  width },
    };
    
    // front face
    //draw3DLine(cube[0], cube[1], red);
    //draw3DLine(cube[1], cube[2], red);
    //draw3DLine(cube[2], cube[3], red);
    //draw3DLine(cube[3], cube[0], red);
    draw3DRect(width, 0, col);
    

    // back face
    //draw3DLine(cube[4], cube[5], red);
    //draw3DLine(cube[5], cube[6], red);
    //draw3DLine(cube[6], cube[7], red);
    //draw3DLine(cube[7], cube[4], red);
    draw3DRect(width, width, col);

    // connect faces
    //draw3DLine(cube[0], cube[4], col);
    //draw3DLine(cube[1], cube[5], col);
    //draw3DLine(cube[2], cube[6], col);
    //draw3DLine(cube[3], cube[7], col);
}
