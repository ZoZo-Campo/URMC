"""Versioned map bundles: occupancy + SLAM pose graph + route, never overwrite."""
import io,json,re,zipfile
from pathlib import Path
import numpy as np
from urmc_core.enclosure_map import Grid

def slug(value):
    if not isinstance(value,str) or not re.fullmatch(r'[a-zA-Z0-9][a-zA-Z0-9_-]{0,63}',value):raise ValueError('Nom : lettres, chiffres, tirets, 64 caractères maximum.')
    return value

class MapStore:
    def __init__(self,root):self.root=Path(root);self.root.mkdir(parents=True,exist_ok=True)
    def folder(self,name):
        path=self.root/slug(name)
        if path.is_symlink() or path.resolve().parent!=self.root.resolve():raise ValueError('Chemin de carte refusé.')
        return path
    def list(self):
        rows=[]
        for p in sorted(self.root.iterdir(),reverse=True):
            if p.name.startswith('.') or not p.is_dir() or p.is_symlink():continue
            try:meta=json.loads((p/'metadata.json').read_text());rows.append(dict(name=p.name,created=meta['created'],robot_id=meta['robot_id'],navigation_mode=meta.get('navigation_mode','slam'),has_graph=(p/'slam.posegraph').exists() and (p/'slam.data').exists()))
            except (OSError,ValueError,KeyError):continue
        return rows
    def load(self,name):
        p=self.folder(name);meta=json.loads((p/'metadata.json').read_text())
        with np.load(p/'grid.npz',allow_pickle=False) as data:grid=Grid(data['data'],float(data['resolution']),data['origin'].tolist())
        return grid,meta
    def write_grid(self,p,grid,meta):
        np.savez_compressed(p/'grid.npz',data=grid.data,resolution=grid.res,origin=grid.origin)
        (p/'preview.png').write_bytes(grid.png())
        pix=np.full(grid.data.shape,205,np.uint8);pix[grid.data==0]=254;pix[grid.data>=50]=0
        (p/'map.pgm').write_bytes(f'P5\n{grid.w} {grid.h}\n255\n'.encode()+np.flipud(pix).tobytes())
        (p/'map.yaml').write_text('image: map.pgm\nresolution: '+str(grid.res)+'\norigin: '+json.dumps(grid.origin)+'\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
        self.metadata(p,meta)
    def metadata(self,p,meta):
        temp=p/'metadata.tmp';temp.write_text(json.dumps(meta,ensure_ascii=False,indent=2));temp.replace(p/'metadata.json')
    def export(self,name):
        p=self.folder(name);self.load(name);stream=io.BytesIO()
        with zipfile.ZipFile(stream,'w',zipfile.ZIP_DEFLATED) as z:
            for file in p.iterdir():
                if file.is_file() and not file.is_symlink() and file.suffix in ('.json','.npz','.png','.pgm','.yaml','.posegraph','.data'):z.write(file,name+'/'+file.name)
        if stream.tell()>64_000_000:raise ValueError('Archive trop volumineuse.')
        return stream.getvalue()
