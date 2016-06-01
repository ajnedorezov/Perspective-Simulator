savedIPM = load('@IPM\myIPM.mat');
self.aVars.IPM = savedIPM.myIPM;

[xx,yy] = meshgrid(1:10:640,self.aVars.IPM.rHorizon:10:480);
pts = self.aVars.IPM.transformSinglePoint(yy,xx);
clf(figure(99)), subplot(211), plot(xx,yy,'o'), axis equal, hold on, 
subplot(212), plot(pts(1,:),pts(2,:),'x'), axis equal, hold on

pts = self.aVars.IPM.transformSinglePoint(400,500);
subplot(211), plot(500,400,'s', 'markersize', 6, 'linewidth', 3)
subplot(212), plot(pts(1,:),pts(2,:),'s', 'markersize', 6, 'linewidth', 3)