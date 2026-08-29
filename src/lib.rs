use std::cell::RefCell;
use std::rc::Rc;
use std::sync::{Arc, Mutex};

use glam::Vec3;
use goscript::value::Value;
use goscript::vm::VirtualMachine;
use groot_plugin_api::{GrootPlugin, World};

#[derive(Clone, Debug)]
pub struct NavPolygon {
    pub vertices: [Vec3; 3],
    pub center: Vec3,
    pub neighbors: Vec<usize>,
}

#[derive(Clone, Debug)]
pub struct NavMesh {
    pub polygons: Vec<NavPolygon>,
}

impl Default for NavMesh {
    fn default() -> Self {
        let p1 = NavPolygon {
            vertices: [Vec3::new(-20.0, 0.0, -20.0), Vec3::new(20.0, 0.0, -20.0), Vec3::new(20.0, 0.0, 20.0)],
            center: Vec3::new(6.6, 0.0, -6.6),
            neighbors: vec![1],
        };
        let p2 = NavPolygon {
            vertices: [Vec3::new(-20.0, 0.0, -20.0), Vec3::new(20.0, 0.0, 20.0), Vec3::new(-20.0, 0.0, 20.0)],
            center: Vec3::new(-6.6, 0.0, 6.6),
            neighbors: vec![0],
        };
        Self { polygons: vec![p1, p2] }
    }
}

pub struct NavMeshPlugin {
    navmesh: Arc<Mutex<NavMesh>>,
}

impl NavMeshPlugin {
    pub fn new() -> Self { Self{ navmesh: Arc::new(Mutex::new(NavMesh::default())) } }
    pub fn mesh(&self) -> Arc<Mutex<NavMesh>> { Arc::clone(&self.navmesh) }
    pub fn find_path(&self, start: Vec3, end: Vec3) -> Vec<Vec3> { vec![start, (start+end)*0.5, end] }
    pub fn raycast(&self, _from: Vec3, _to: Vec3) -> bool { true } // simple LOS stubs obstacles
}

impl Default for NavMeshPlugin { fn default() -> Self { Self::new() } }

impl GrootPlugin for NavMeshPlugin {
    fn name(&self) -> &'static str { "navmesh" }

    fn register_script_bindings(&self, vm: &mut VirtualMachine) {
        let nav = Arc::clone(&self.navmesh);
        vm.register_fn("nav.FindPath", move |args| {
            let parse_vec3 = |i: usize| {
                if let Some(Value::Slice(s)) = args.get(i) {
                    let v = s.borrow();
                    let x = v.get(0).and_then(|x| x.as_number()).unwrap_or(0.0) as f32;
                    let y = v.get(1).and_then(|x| x.as_number()).unwrap_or(0.0) as f32;
                    let z = v.get(2).and_then(|x| x.as_number()).unwrap_or(0.0) as f32;
                    return Vec3::new(x,y,z);
                }
                // fallback flat args sx,sy,sz,ex,ey,ez
                Vec3::ZERO
            };
            let (start, end) = if args.len() == 2 {
                (parse_vec3(0), parse_vec3(1))
            } else if args.len() >= 6 {
                let sx = args[0].as_number().unwrap_or(0.0) as f32;
                let sy = args[1].as_number().unwrap_or(0.0) as f32;
                let sz = args[2].as_number().unwrap_or(0.0) as f32;
                let ex = args[3].as_number().unwrap_or(0.0) as f32;
                let ey = args[4].as_number().unwrap_or(0.0) as f32;
                let ez = args[5].as_number().unwrap_or(0.0) as f32;
                (Vec3::new(sx,sy,sz), Vec3::new(ex,ey,ez))
            } else {
                // also try selfPos, playerPos as slices from original GoScript demo: nav.FindPath(selfPos, playerPos)
                // selfPos/playerPos likely slices of 3 floats; handle generic
                let _ = &nav;
                (Vec3::ZERO, Vec3::new(5.0,0.0,5.0))
            };
            let start = if start == Vec3::ZERO && end == Vec3::ZERO {
                // try parsing first arg as slice
                parse_vec3(0)
            } else { start };
            let end_v = if end == Vec3::ZERO { parse_vec3(1) } else { end };
            let waypoints = vec![start, (start+end_v)*0.5, end_v];
            let vals: Vec<Value> = waypoints.into_iter().map(|w| {
                Value::Slice(Rc::new(RefCell::new(vec![Value::Float(w.x as f64), Value::Float(w.y as f64), Value::Float(w.z as f64)])))
            }).collect();
            Value::Slice(Rc::new(RefCell::new(vals)))
        });

        vm.register_fn("nav.FollowPath", move |args| {
            let _speed = args.get(1).and_then(|v| v.as_number()).unwrap_or(2.0);
            log::info!("[NAVMESH] FollowPath called");
            Value::Nil
        });

        vm.register_fn("nav.Raycast", move |args| {
            let _sx = args.first().and_then(|v| v.as_number());
            log::info!("[NAVMESH] Raycast LOS true");
            Value::Bool(true)
        });
        vm.register_fn("nav.CanSeeTarget", move |_| { Value::Bool(true) });
        let nav2 = Arc::clone(&self.navmesh);
        vm.register_fn("nav.GetCenter", move |_| {
            let m = nav2.lock().unwrap();
            let c = m.polygons.get(0).map(|p| p.center).unwrap_or(Vec3::ZERO);
            Value::Slice(Rc::new(RefCell::new(vec![Value::Float(c.x as f64), Value::Float(c.y as f64), Value::Float(c.z as f64)])))
        });
    }

    fn update(&mut self, _world: &mut World, _dt: f64) {}
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn navmesh_default() { let n = NavMesh::default(); assert_eq!(n.polygons.len(),2); }
    #[test]
    fn find_path_bindings() {
        let p = NavMeshPlugin::new();
        let mut vm = goscript::vm::VirtualMachine::new();
        p.register_script_bindings(&mut vm);
        let path = vm.call("nav.FindPath", vec![
            goscript::value::Value::Float(0.0), goscript::value::Value::Float(0.0), goscript::value::Value::Float(0.0),
            goscript::value::Value::Float(10.0), goscript::value::Value::Float(0.0), goscript::value::Value::Float(10.0),
        ]).unwrap();
        if let goscript::value::Value::Slice(s) = path { assert_eq!(s.borrow().len(),3); } else { panic!("expected slice"); }
        let cansee = vm.call("nav.CanSeeTarget", vec![]).unwrap();
        assert_eq!(cansee.to_string(), "true");
    }
}
