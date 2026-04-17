import torch
from isaaclab.utils.warp import raycast_mesh

def check_line_of_sight(
    env,
    robot_positions: torch.Tensor,
    threat_positions_w: torch.Tensor,
    max_distance: float = 20.0,
) -> tuple[torch.Tensor, torch.Tensor]:
    """
    Computes obstacle-aware line-of-sight (LoS) between robots and threats.
    The function uses the existing lidar ray hits: if an obstacle hit along the threat
    bearing is closer than the threat itself, the LoS is considered blocked.
    
    Args:
        robot_positions: Base positions of robots [num_envs, 3]
        threat_positions_w: Threat positions in world frame [num_envs, num_threats, 3]
        max_distance: Maximum sight distance
        
    Returns:
        exposure_mask: [num_envs, num_threats] Boolean tensor (True if exposed)
        blocked_mask: [num_envs, num_threats] Boolean tensor (True if blocked by obstacle)
    """
    sensor = env.scene.sensors["lidar"]
    mesh_path = sensor.cfg.mesh_prim_paths[0]
    warp_mesh = sensor.__class__.meshes.get(mesh_path, None)

    # Fallback to exposed in-range if mesh is unavailable.
    seg = robot_positions.unsqueeze(1) - threat_positions_w
    seg_len = torch.norm(seg, dim=-1).clamp(min=1e-6)
    in_range = seg_len < max_distance
    if warp_mesh is None:
        blocked_mask = torch.zeros_like(in_range)
        exposure_mask = in_range
        return exposure_mask, blocked_mask

    # Raycast from threat to robot against environment mesh.
    ray_starts = threat_positions_w
    ray_dirs = seg / seg_len.unsqueeze(-1)
    ray_hits, _, _, _ = raycast_mesh(ray_starts, ray_dirs, mesh=warp_mesh, max_dist=max_distance)

    hit_dist = torch.norm(ray_hits - ray_starts, dim=-1)
    hit_dist = torch.nan_to_num(hit_dist, nan=float("inf"), posinf=float("inf"), neginf=float("inf"))

    # If first hit is sufficiently before robot, LoS is blocked.
    blocked_mask = in_range & (hit_dist + 0.20 < seg_len)
    exposure_mask = in_range & (~blocked_mask)
    return exposure_mask, blocked_mask
