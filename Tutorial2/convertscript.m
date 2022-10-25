f=dir('*png;')
fil={f.name};
for k=l:numel(fil)
    file=fil{k};
    new_file=strrep(strcat('I',file),'.png','.jpg');
    im=imread(file);
    imwrite(im,new_file);
end 