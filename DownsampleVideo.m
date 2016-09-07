vid = VideoReader('To Work.mov');

newVid = VideoWriter('Downsampled To Work Video 2.avi');
open(newVid);

for n = (30*3*60):(30*7*30)
    im = read(vid, n);
    im = imresize(im, 0.5);
    
    
    writeVideo(newVid, im);
    disp(n);
end

close(newVid);
