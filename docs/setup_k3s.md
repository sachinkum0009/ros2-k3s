# Setup k3s

Setting up k3s on Server as well as Clients

```bash
# Server Node
curl -sfL https://get.k3s.io | sh -

# test k3s
k3s kubectl get nodes

# if it gives error reading config, use below cmd
sudo chmod 644 /etc/rancher/k3s/k3s.yaml

# Get token value from `/var/lib/rancher/k3s/server/node-token`

# Client Node
curl -sfL https://get.k3s.io | K3S_URL=https://myserver:6443 K3S_TOKEN=mynodetoken sh -

# Verify client nodes
k3s kubectl get nodes
```
