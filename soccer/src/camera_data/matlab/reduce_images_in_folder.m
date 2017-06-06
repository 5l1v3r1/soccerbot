function reduce_images_in_folder()
    folders = get_folders('../training_images/');
    
    for i=1:length(folders)
       reduce_in_folder(strcat(folders(i).folder,'/',folders(i).name));
    end
end

function folders = get_folders(folder)
    folders = dir(folder);
    folders = folders([folders.isdir]);
    folders = folders(arrayfun(@(x) x.name(1), folders) ~= '.');
end

function reduce_in_folder(folder)
    if contains(folder, '160x120')
        return
    end

    imgs = dir(strcat(folder, '/*.jpg'));
    newdir = strcat(strcat(folder, '160x120'));
    
    if ~exist(newdir, 'file')
       mkdir(newdir);
    end
    
    n = length(imgs);

    for i=1:n
       currentfilename = imgs(i).name;
       newfilename = strcat(int2str(i), '.png');
       currentImage = imread(strcat(folder, '/', currentfilename));
       resizedImage = imresize(currentImage, 0.5);
       imwrite(resizedImage, fullfile(newdir, newfilename));
    end
end