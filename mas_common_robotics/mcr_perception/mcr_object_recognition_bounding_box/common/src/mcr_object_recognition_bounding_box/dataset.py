import os
import yaml
import numpy as np


class Dataset:
    def __init__(self, base_folder, dataset_name):
        self.base_folder = base_folder
        self.dataset_name = dataset_name
        self.dataset_folder = os.path.join(base_folder, dataset_name)
        if not os.path.isdir(self.dataset_folder):
            os.mkdir(self.dataset_folder)
        self.objects_yaml = os.path.join(base_folder, 'objects.yaml')
        self.objects = list(yaml.load_all(open(self.objects_yaml)))

    def store(self, object_id, dim, pts, color_mean, color_median):
        f = open(self.object_data_filename(object_id), 'a')
        d = sorted([dim.x, dim.y, dim.z], reverse=True)
        f.write('%.3f %.3f %.3f %i %.2f %.2f\n' % (d[0], d[1], d[2], pts,
                                                   color_mean, color_median))
        f.close()

    def object_data_filename(self, object_id):
        return os.path.join(self.dataset_folder, object_id + '.dat')

    def load(self, objects='all', start_index=0):
        if objects == 'all':
            objects = [obj['id'] for obj in self.objects]
        elif isinstance(objects, str):
            objects = [objects]
        X = []
        Y = []
        object_ids = []
        for obj in objects:
            try:
                i = len(object_ids) + start_index
                x = np.atleast_2d(np.loadtxt(self.object_data_filename(obj)))
                y = (np.ones((len(x), 1)) * i).astype(np.int)
                object_ids.append(obj)
                X.append(x)
                Y.append(y)
            except IOError:
                print 'Unable to read file for object "%s" in dataset "%s"' % (
                       obj, self.dataset_name)
                pass
        if object_ids:
            try:
                return (np.vstack(X), np.vstack(Y), object_ids)
            except ValueError:
                print 'Unable to concatenate arrays, different number of features.'
        return (np.array([]), np.array([]), [])


class JointDataset:
    def __init__(self, datasets):
        self.datasets = datasets
        self.objects = datasets[0].objects

    def load(self, objects='all', start_index=0):
        if objects == 'all':
            objects = [obj['id'] for obj in self.objects]
        elif isinstance(objects, str):
            objects = [objects]
        X = []
        Y = []
        object_ids = []
        for obj in objects:
            i = len(object_ids) + start_index
            loaded = False
            for ds in self.datasets:
                x, y, labels = ds.load(obj, i)
                if labels:
                    X.append(x)
                    Y.append(y)
                    loaded = True
            if loaded:
                object_ids.append(obj)
        try:
            return (np.vstack(X), np.vstack(Y), object_ids)
        except ValueError:
            print 'Unable to concatenate arrays, different number of features.'
            return (np.array([]), np.array([]), [])
