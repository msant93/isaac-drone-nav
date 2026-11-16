# Running the project on AWS on a low budget

A step-by-step guide to a **pay-only-when-you-use-it** GPU workstation on AWS
for testing the project in Isaac Sim.

> Prices are approximate us-east-1 Linux rates (Nov 2026) from third-party
> trackers. Check the current prices in the
> [AWS pricing calculator](https://calculator.aws/) before you start.
> Items marked ⚠ could not be confirmed against NVIDIA's documentation while
> this guide was written; each one says how to check it.

---

## 0. Cost-saving strategy (read this first)

1. **Use AWS only for Isaac Sim runs.** Most development (Phases 2–5, 8, 9
   and the unit tests) runs on your own laptop without a GPU.
2. **Pay per hour, and stop the instance whenever you are not running something.**
   A *stopped* instance costs nothing for compute; you only pay for its disk.
3. **Use the smallest instance that works:**

   | Instance | GPU (RT cores) | vCPU / RAM | On-demand | Spot (varies) | Role |
   |---|---|---|---|---|---|
   | **`g4dn.2xlarge`** | Tesla T4, 16 GB | 8 / 32 GB | ~$0.75/h | ~$0.25–0.32/h | **Default: cheapest one that fits** |
   | `g6.2xlarge` | L4, 24 GB | 8 / 32 GB | ~$0.98/h | ~$0.42–0.67/h | Fallback if the T4 is too slow or unsupported |
   | `g5.2xlarge` | A10G, 24 GB | 8 / 32 GB | ~$1.21/h | varies | Not needed for this project |

   Do **not** use `g4dn.xlarge`: 16 GB of RAM is not enough for Isaac Sim.
   Do not use `p3`/`p4`/`p5` either: those GPUs have no RT cores.

   ⚠ The T4 is a Turing GPU. Older Isaac Sim AWS guides recommend
   `g4dn.2xlarge` as the minimum, but NVIDIA's official minimum for 5.x is a
   much stronger GPU. Expect slow rendering on the T4. Physics and headless
   tests should work. If Isaac Sim 5.1 refuses to start or is unusably slow,
   switch the instance type to `g6.2xlarge` (Section 9); your disk and work
   are kept. Check: the *Requirements* page of the Isaac Sim 5.1.0 docs.
4. **Keep the disk small** (100 GB gp3 ≈ $8/month). The disk is the only cost
   while the instance is stopped.
5. **Use an auto-stop alarm** (Section 6), so a forgotten instance stops by itself.
6. **Set a budget alert** (Section 1), so you hear about unexpected costs early.

**Estimated cost for the whole project:** about 30–40 GPU hours ×
$0.75 ≈ $25–30, plus disk ≈ $8/month for about 3 months ≈ $25. In total roughly
**$50–60**, or less with Spot. Ask your university whether it offers AWS
credits (e.g. through *AWS Academy* or a research credits programme).

---

## 1. Create and secure the AWS account (once)

1. Create an account at <https://aws.amazon.com/>. A credit card is required;
   the free tier does **not** cover GPU instances.
2. Enable **MFA** on the root user: *IAM → Security credentials*.
3. **Budget alert:** go to *Billing and Cost Management → Budgets → Create budget*,
   choose *Monthly cost budget*, enter e.g. **$20**, and add your email. You get
   an email at 85% and 100%.
4. Choose a **region** and use it for everything. `us-east-1` (N. Virginia) is
   usually the cheapest and has the best GPU availability. From Europe,
   `eu-west-1` (Ireland) has lower latency for the GUI stream but costs a bit
   more. The region selector is at the top right of the console.

## 2. Request GPU quota (do it NOW: approval can take 1–3 days)

New accounts usually have **0 vCPUs** allowed for GPU instances.

1. Open *Service Quotas → AWS services → Amazon Elastic Compute Cloud (Amazon EC2)*.
2. Search **"Running On-Demand G and VT instances"**, then *Request increase at
   account level*, and enter **8**. This covers one 8-vCPU instance.
3. (Optional, for Spot) Search **"All G and VT Spot Instance Requests"** and
   request **8**.
4. In the description write something like: *"University project: robotics
   simulation with NVIDIA Isaac Sim on one g4dn.2xlarge instance."*

Wait for the approval email before continuing with Section 4.

## 3. Create an SSH key pair (once)

1. *EC2 → Network & Security → Key Pairs → Create key pair*. Name it
   `isaac-key`, type **ED25519**, format **.pem**.
2. The file downloads. Move it and protect it on your computer:
   ```bash
   mkdir -p ~/.ssh && mv ~/Downloads/isaac-key.pem ~/.ssh/ && chmod 400 ~/.ssh/isaac-key.pem
   ```
   On Windows, use PowerShell's built-in `ssh` or WSL; the `chmod` step only
   applies on Linux/macOS/WSL.

## 4. Launch the instance

*EC2 → Instances → Launch instances*:

| Field | Value |
|---|---|
| Name | `isaac-drone-nav` |
| AMI | Search for **"Deep Learning Base OSS Nvidia Driver GPU AMI (Ubuntu 22.04)"** (published by Amazon, **free**). It ships the NVIDIA driver, Docker and the NVIDIA Container Toolkit preinstalled. |
| Instance type | `g4dn.2xlarge` |
| Key pair | `isaac-key` |
| Network | *Create security group*: allow **SSH from "My IP"** only |
| Storage | **100 GiB gp3** (root volume) |
| Advanced → Shutdown behaviour | **Stop** (the default) |

Avoid the NVIDIA "Isaac Sim" AWS Marketplace AMIs for this project. They can
add software charges and may only support larger instance types (⚠ unverified).

**Spot option (≈ 60% cheaper).** Under *Advanced details*, tick *Request Spot
Instances*, then choose request type **Persistent** and interruption behaviour
**Stop**. AWS may stop your instance with 2 minutes' notice when it needs the
capacity back; your disk is kept, and you start it again later. Trackers
report interruption rates above 20% for `g4dn.2xlarge`, so use Spot for
headless batch runs and On-demand when you need an uninterrupted session.
Tip: do the first setup on On-demand.

Click **Launch instance**. After about 1 minute it is *Running*. Copy its **Public IPv4 address**.

## 5. First connection and setup (about 30–45 min, once)

```bash
ssh -i ~/.ssh/isaac-key.pem ubuntu@<PUBLIC_IP>
```

Check the GPU, Docker and the toolkit:
```bash
nvidia-smi
```
```bash
docker run --rm --gpus all ubuntu nvidia-smi
```
Both must list the **Tesla T4**. ⚠ Note the driver version and compare it with
the "Linux driver" row of the Isaac Sim 5.1.0 *Requirements* page. If the
driver is too old, tell me the version and we will pick a newer AMI.

If `docker` needs `sudo`, run this, then log out and back in:
```bash
sudo usermod -aG docker ubuntu
```

Get the project and check the machine:
```bash
git clone https://github.com/msant93/isaac-drone-nav.git && cd isaac-drone-nav
```
```bash
python3 scripts/check_env.py
```

Build the image. This downloads about 20 GB from NVIDIA and takes 10–20 minutes
the first time. The image stays on the disk across stop/start.
```bash
make docker-build
```
If the pull fails with `unauthorized`, log in to NGC first (free account,
API key from <https://ngc.nvidia.com/setup/api-key>):
```bash
docker login nvcr.io
```
Use the username `$oauthtoken` (literally) and your API key as the password.

Run the smoke test (headless). The **first start compiles shaders and can take
5–15 minutes**; the next starts are much faster thanks to `docker/cache/`.
```bash
make smoke
```

Copy the results to your laptop (run this **on your laptop**):
```bash
scp -i ~/.ssh/isaac-key.pem -r ubuntu@<PUBLIC_IP>:~/isaac-drone-nav/results/smoke_test ./
```

## 6. Auto-stop when idle (strongly recommended, 5 minutes)

So you never pay for a night you forgot about:

1. Select the instance, open the **Monitoring** tab, and click the *+* on
   **CPU utilization** (or *Manage CloudWatch alarms → Create*).
2. Condition: **CPU utilization < 5% for 6 consecutive periods of 5 minutes**
   (30 minutes).
3. Alarm action: **Stop this instance**.
4. Create it.

Isaac Sim keeps the CPU busy while it runs, so the alarm only fires when the
machine is really idle.

## 7. Daily workflow

**Start:** *EC2 → Instances →* select it, then *Instance state → Start*. **The
public IP changes at every start**: copy the new one.

**Edit code:** two options.
- **VS Code Remote-SSH** (recommended): install the *Remote - SSH* extension,
  then *Connect to Host* `ubuntu@<PUBLIC_IP>` with the key. You edit files
  directly on the instance.
- Or work on your laptop, push to GitHub, and run `git pull` on the instance.

**Run:**
```bash
cd ~/isaac-drone-nav && git pull && make smoke
```

**Stop when you finish**, either from the console (*Instance state → Stop*)
or from the SSH session:
```bash
sudo shutdown -h now
```
Never choose **Terminate** unless you want to delete everything.

## 8. Seeing the 3D view (only when you need it)

Headless runs plus CSV/plots are enough for most testing. To watch the drone:

1. Start the run with the livestream script:
   ```bash
   ./docker/run_livestream.sh scripts/smoke_test_hover.py
   ```
2. In the security group, add inbound rules **from "My IP" only**: TCP **49100**
   and UDP **47998**. ⚠ Check the port list on the *Livestream Clients* page
   of the Isaac Sim 5.1.0 docs.
3. On your laptop, open the **Isaac Sim WebRTC Streaming Client** (download link
   on the same docs page) and enter the instance's public IP.
4. Remove the two rules when you are done.

For a demo video you do not need the stream: in Phase 12 the video is recorded
headless, directly to a file.

## 9. Changing instance type (e.g. T4 → L4)

Stop the instance, then *Actions → Instance settings → Change instance type*:
choose `g6.2xlarge` and start it again. The disk, the Docker image and your
files are kept. If you use the On-demand quota, make sure it covers 8 vCPUs.

## 10. What costs money (checklist)

| Item | Cost | How to avoid it |
|---|---|---|
| Running instance | $/hour while *Running* | Stop it; auto-stop alarm |
| EBS disk | ≈ $0.08/GB-month, even when stopped | Keep it at 100 GB; delete it at the end |
| Public IPv4 | ≈ $0.005/h while running | Nothing to do (free while stopped). **Don't** allocate an Elastic IP: it is charged while the instance is stopped |
| Snapshots / AMIs you create | ≈ $0.05/GB-month | Don't create them, or delete them |
| Data transfer out | First 100 GB/month free | Livestream only when needed |

**End of the project:** *Terminate* the instance, then check *EC2 → Volumes*
and *EC2 → Snapshots* and delete anything left. After that the cost is $0.

## 11. Troubleshooting

| Problem | Fix |
|---|---|
| Launch fails with `VcpuLimitExceeded` | Quota not approved yet (Section 2) |
| `InsufficientInstanceCapacity` | Try another Availability Zone (subnet) or region, or try again later |
| SSH times out | Your IP changed: edit the security group rule (*My IP*) |
| `Permission denied (publickey)` | Use the user `ubuntu` and `-i ~/.ssh/isaac-key.pem`, with `chmod 400` on the key |
| `could not select device driver "nvidia"` | `sudo systemctl restart docker`; check `nvidia-ctk --version` |
| `no space left on device` | `docker system prune` and remove old images; or grow the volume (*EC2 → Volumes → Modify*) |
| Isaac Sim crashes on start on the T4 | Send me the log; fall back to `g6.2xlarge` (Section 9) |
| Spot instance stopped by itself | Capacity was reclaimed: start it again later, or switch to On-demand |
