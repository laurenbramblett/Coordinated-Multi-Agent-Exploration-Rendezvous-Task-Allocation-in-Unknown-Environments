function rzDecision = rzDecide(pthToRz,varEps,M0,timeInEx,compTime,eV)
    rzLength = length(pthToRz);
    pthInd = sub2ind(size(M0),pthToRz(:,1),pthToRz(:,2));
    known = sum(M0(pthInd)<0.15);
    unk = rzLength-known;
    rzDecision = rzLength + varEps*unk<timeInEx+compTime-eV;
end
    