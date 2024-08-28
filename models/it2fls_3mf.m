function y =it2fls_3mf(inputs)
    NumInputs=2;
    NumMF=3;
    NumRules=9;
    NumData=size(inputs,1);
    Nodes=zeros(36,2);
    MFparams=[0.16207     0.18457      1.0085    -0.43013     0.37617;
    0.072534    0.091516      1.0262    -0.02401      0.3882;
    0.022101     0.02803      1.0275     0.36549     0.50255;
    1.3126      1.5771     0.99528     -4.6208     0.54202;
    1.278      1.5463      1.0527     -1.3363     0.40656;
    1.2884      1.5522      1.0617      1.9235     0.42364];
    Cparams=[1.6165    0.043378      0.3826;
    1.0109  -0.0075928    0.047905;
    1.3223   -0.032349  -0.0010639;
    3.7929   -0.072392    -0.40528;
    3.0834    -0.36098    -0.67003;
    1.0144    -0.70076    -0.11468;
    2.4935     0.10978    -0.61202;
    2.9009    -0.90953    -0.95173;
    0.26377     0.26037     0.53858];
    Rules=[3  6;
    3  7;
    3  8;
    4  6;
    4  7;
    4  8;
    5  6;
    5  7;
    5  8];

    for i=1:NumData
        Nodes(1:NumInputs,1)=inputs(i,:)';
        Nodes(1:NumInputs,2)=inputs(i,:)';

        for j=1:NumInputs
            for k=1:NumMF
                ind=NumInputs+(j-1)*NumMF+k;
                x = Nodes(j,1);
                b = MFparams((j-1)*NumMF+k,3);
                c = MFparams((j-1)*NumMF+k,4);
                ai = MFparams((j-1)*NumMF+k,1);
                tmp1i = (x - c)/ai;
                if tmp1i == 0
                    tmp2i=0;
                else
                    tmp2i = (tmp1i*tmp1i)^b;
                end
                Nodes(ind,1)=MFparams((j-1)*NumMF+k,5)/(1+ tmp2i);
                as = MFparams((j-1)*NumMF+k,2);
                tmp1s = (x - c)/as;
                if tmp1s == 0
                    tmp2s=0;
                else
                    tmp2s = (tmp1s*tmp1s)^b;
                end
                Nodes(ind,2)=1/(1+ tmp2s);
            end
        end

        st=NumInputs+NumInputs*NumMF;
        for j=st+1:st+NumRules
            tmpi=cumprod(Nodes(Rules(j-st,:)',1));
            tmps=cumprod(Nodes(Rules(j-st,:)',2));
            Nodes(j,1)=tmpi(end);
            Nodes(j,2)=tmps(end);
        end

        st=NumInputs+NumInputs*NumMF;
        wi=Nodes(st+1:st+NumRules,1)';
        ws=Nodes(st+1:st+NumRules,2)';
        [wi,O1]=sort(wi);
        ws=sort(ws);
        wn = (wi+ws)/2;
        yi=sum(wi.*wn)/sum(wn);
        ys=sum(ws.*wn)/sum(wn);
        for j=1:NumRules-1
            if wi(j)<=yi && yi<=wi(j+1)
               l=j;
            end
            if ws(j)<=ys && ys<=ws(j+1)
               r=j;
            end
        end
        Xl=[ws(1:l) wi(l+1:NumRules)]/sum([ws(1:l) wi(l+1:NumRules)]);
        Xr=[wi(1:r) ws(r+1:NumRules)]/sum([wi(1:r) ws(r+1:NumRules)]);
        X=(Xl+Xr)/2;
        X=X(O1);
        st=NumInputs+NumInputs*NumMF+NumRules;
        Nodes(st+1:st+NumRules,1)=X;
        Nodes(st+1:st+NumRules,2)=X;

        st=NumInputs+NumInputs*NumMF+2*NumRules;
        inp=Nodes(1:NumInputs,1)';
        for j=1:NumRules
            wn=Nodes(j+st-NumRules,1);
            Nodes(j+st,1)=wn*(sum(Cparams(j,1:end-1).*inp)+Cparams(j,end));
        end

        Nodes(end,1)=sum(Nodes(end-NumRules:end-1,1));
        y(i,1)=Nodes(end,1);
    end
end
