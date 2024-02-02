<script>
import {Command} from '@tauri-apps/api/shell'

let cmdPromise = testCmd();

async function testCmd() {
    const cmd = Command.sidecar('bin/ros2-querier');
    const child = await cmd.execute();
    console.log("done?")
    return child.stdout;
}
</script>

<p>
    {#await cmdPromise}
       Running cmd... 
    {:then cmd} 
        {cmd}
    {:catch err}
        Error: {err.message}
    {/await}
</p>
<h1>Welcome to SvelteKit</h1>
<p>Visit <a href="https://kit.svelte.dev">kit.svelte.dev</a> to read the documentation</p>
